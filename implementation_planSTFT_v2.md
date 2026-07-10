# Plano de Implementação v2: STFT com Compensação de Transport Delay + Auto-Calibrador

> Substitui o `implementation_planSTFT.md` (v1). A v2 corrige 9 problemas encontrados na auditoria
> da v1 contra o código real do repositório e muda a arquitetura de controle com base em como
> MoTeC, Emtron, Link e MegaSquirt resolvem o mesmo problema.

---

## 1. Auditoria da v1 — o que estava errado

| # | Problema da v1 | Evidência no código | Correção na v2 |
|---|---|---|---|
| 1 | **O gate de transport delay não faz o que promete.** `cell.update()` integra `erro × dt/τ` com `dt = 5 ms` fixo (`integrator_dt = FAST_CALLBACK_PERIOD_MS * 0.001f`, 200 Hz). Chamar isso 1× por janela de delay só reduz o ganho efetivo pelo fator `5ms/delay` — matematicamente idêntico a aumentar τ. Nenhum benefício real de compensação de delay; pior, a taxa de correção passa a depender da tabela de delay de forma não-óbvia. | `firmware/controllers/math/closed_loop_fuel_cell.cpp:7,29` | Novo controlador **"passo periódico"** (Período + Ganho), seção 4. É o modelo MoTeC/Link de verdade: passo `ganho × erro` a cada período completo de resposta, não integração contínua estrangulada. |
| 2 | **`uint8_t stftCalStepPercent` com faixa −20..−5** — uint8 não armazena negativo. | proposta v1 em `rusefi_config.txt` | `int8_t`, faixa −15..−5, e `0 → −10` via `defaultsOrFixOnBurn()` (tunes antigas migram com 0). |
| 3 | **Dupla escala no `timeConstant`.** O campo é `uint16_t autoscale ... 0.1` (`rusefi_config.txt:347`) → no C++ é `scaled_channel`, atribuição recebe valor de engenharia. `timeConstant = lagSec * 10.0f` gravaria 10× o valor. | `rusefi_config.txt:347` | Atribuir valor de engenharia direto: `cfg.cellCfgs[r].timeConstant = seconds;`. |
| 4 | **O passo de calibração contamina o LTFT.** `getCorrection()` alimenta `LongTermFuelTrim::learn()` a cada 5 ms (`engine2.cpp:197-200`); multiplicar o step de −10 % em `result.banks[0]` faz o LTFT "aprender" o degrau forçado como trim. | `firmware/controllers/algo/engine2.cpp:197`, `long_term_fuel_trim.cpp:104` | `LongTermFuelTrim::learn()` ganha early-return quando `ShortTermFuelTrim::isCalibrationActive()`. |
| 5 | **`applyCalibrationResult()` chamada em loop.** Com o estado preso em `Done`, `isDone()` é true a cada 5 ms → escrita na config e `efiPrintf` 200×/s. | proposta v1 de `getCorrection()` | Aplicar exatamente 1× na transição para `Done` e mover a máquina para `Idle` (resultados ficam no live data). |
| 6 | **Detecção de dead time por limiar de 5 % é abaixo do ruído do sensor.** 5 % de Δλ≈0,111 = 0,0055 λ — menor que o ruído típico de wideband (±0,005–0,01) e menor que a própria tolerância de estabilidade da v1 (±0,01). Falso-positivo garantido. | proposta v1 `stft_calibrator.cpp` | **Análise retrospectiva em buffer**: gravar λ num buffer estático durante o degrau e extrair t10/t90 da curva filtrada depois que ela assenta (seção 5). |
| 7 | **Trigger por bit de config (`stftTriggerCalibration`) briga com o sync de páginas do TS.** Firmware limpando um bit que o TS considera "seu" gera estado sujo/burn inconsistente. O padrão do projeto para botões é outro. | `tunerstudio.template.ini:2194-2196` (`cmd_ltft_reset` → `TS_BENCH_CATEGORY` + `bench_mode_e`), handler em `bench_test.cpp:425` | Dois comandos novos em `bench_mode_e`: `STFT_START_CALIBRATION` / `STFT_STOP_CALIBRATION`. |
| 8 | **`m_lambdaStabilityRef` nunca é inicializado em `start()`** — a janela de estabilidade parte de lixo/valor obsoleto. | proposta v1 | `start()` zera todo o estado, inclusive referência de estabilidade. |
| 9 | **Sem guarda de segurança de região/transiente.** Degrau pobre em `Power`/WOT = detonação/EGT; degrau durante transiente de TPS mede lixo. | — | Pré-condições de armamento + condições contínuas de aborto (seção 5.3). |

Duas decisões da v1 também foram **revisadas** (não eram bugs, mas há opção melhor):

- **Eixo da tabela de delay**: RPM×carga 4×4 (16 células) → **curva 1D de 8 pontos vs vazão mássica (kg/h)**.
  É como MoTeC ("lambda delay period versus Exhaust Mass Flow table") e Emtron ("Air Mass Flow Final
  (g/s)") fazem. O delay físico é ≈ `volume do escape ÷ vazão volumétrica` + resposta do sensor — função
  monotônica de **uma** variável. O rusEFI já calcula `engine->engineState.airflowEstimate` (kg/h) a cada
  fast callback (`fuel_math.cpp:208`), antes de `getCorrection()` ser chamado. Metade das células para
  calibrar, sem risco de célula RPM alto/carga baixa ficar órfã.
- **O que a tabela guarda**: em vez de separar Dead Time (tabela) e Lag (timeConstant), a curva guarda o
  **Período completo** (t95 ≈ DeadTime + 3×Lag), que é exatamente o que o controlador de passo periódico
  precisa. Dead Time e Lag continuam medidos e expostos no live data para diagnóstico.

---

## 2. Benchmark — como as outras ECUs fazem

| ECU | Estratégia | O que aproveitamos |
|---|---|---|
| **MoTeC M1 / GPRP** | Dead Time (início da reação) + Period (t95). Correção discreta: a cada Period, aplica `Trim Gain %` do erro. Tabela de Period indexada por **exhaust mass flow**. Calibração: degrau de 5–10 % no Fuel Mixture Aim, medir no log o tempo até o alvo. | Arquitetura inteira do modo novo: Período+Ganho, eixo em vazão mássica, procedimento do auto-calibrador. |
| **Emtron KV** | PID + tabela de "Lambda Transport Delay" vs Air Mass Flow (g/s); o delay corrige a atribuição temporal do feedback do PID. | Confirma o eixo em vazão mássica e a medição por degrau de alvo no log. |
| **Link G4X** | Gain table + **Update Rate table (Hz)** — mesmo conceito move-and-wait: em baixa vazão o loop atualiza mais devagar. | Confirma que taxa de atualização variável com vazão é o padrão da indústria. |
| **MegaSquirt MS3** | Dois algoritmos: "simple" (corrige a cada N eventos de ignição — delay proporcional a RPM de graça) e PID com tabela de EGO delay (ms). Aviso da doc: delay exagerado faz a correção "correr atrás". | Validação de campo do modo passo-periódico; alerta para clamps de período. |
| **OEM / literatura (Bosch, SAE)** | PI com compensação de delay (Smith predictor / adaptive delay-compensated PID). O problema clássico documentado: **acúmulo do integrador durante o dead time** → overshoot/oscilação — exatamente o que o STFT atual do rusEFI tem com τ pequeno. | Justificativa teórica: sem compensação, τ precisa ser ≫ delay (por isso o default atual de 30 s); com compensação, converge em ~3 períodos. |

Fontes no fim do documento.

---

## 3. Etapas de entrega

Duas etapas independentes, cada uma compilável, testável e utilizável sozinha. A Etapa 2 depende da
Etapa 1 (escreve na curva criada lá); nenhum campo da Etapa 1 muda na Etapa 2.

### Etapa 1 — Curva de transport delay + controlador "Periodic Step"

Arquitetura na seção 4; arquivos nas seções 6.1, 6.2, 6.4, 6.5, 6.9, 6.10, 6.11, 6.12 (partes
marcadas como Etapa 1).

- **Config**: `STFT_PERIOD_CURVE_SIZE`, curva `correctionPeriodFlowBins`/`correctionPeriodMs`,
  `correctionAlgorithm`, `trimStepGain`, `maxStepPercent`. (`stftCalStepPercent` fica para a
  Etapa 2 — novo append no final de `stft_s`, rotina.)
- **Live data**: só `stftPeriodMs`.
- **Código**: `applyStep()` na célula, despacho por algoritmo + passo periódico em `getCorrection()`,
  defaults e migração `defaultsOrFixOnBurn()`.
- **INI**: curva + seletor de algoritmo/ganhos no `fuelClosedLoopDialog`.
- **Testes**: `PeriodicStep_*` + `Legacy_BitForBitRegression`.

**Utilizável sem a Etapa 2**: a curva é preenchida manualmente pelo método MoTeC/Emtron — em cada
ponto de vazão estável, aplicar um degrau no alvo de λ (ou um degrau de trim) e medir no log o tempo
do comando até o λ assentar; gravar esse tempo (ms) no bin de vazão correspondente. 3 pontos ancoram
a curva, o resto interpola.

**Critério de conclusão**: testes de regressão do modo legado passam sem alteração; no modo novo,
perturbação de 10 % no VE converge em <2 s no log, sem oscilação sustentada.

### Etapa 2 — Auto-calibração do transport delay

Arquitetura na seção 5; arquivos nas seções 6.1 (resto), 6.2 (resto), 6.3, 6.5 (resto), 6.6, 6.7,
6.8, 6.11 (resto).

- **Config**: `stftCalStepPercent` (int8, append no final de `stft_s`).
- **Live data**: `stftCalState`, `stftCalAbortReason`, `stftCalDeadTimeMs`, `stftCalLagMs`,
  `stftCalPeriodMs`.
- **Código**: `stft_calibrator.{h,cpp}` novo, integração no `getCorrection()`, freeze do LTFT,
  `bench_mode_e` + handler no `bench_test.cpp`, `math.mk`.
- **INI**: dialog de calibração, 2 commandButtons, gaugeCategory.
- **Testes**: planta FOPDT simulada + `Calibrator_*` + `Ltft_FrozenDuringCalibration`.

**Critério de conclusão**: calibração no simulador/bancada preenche o bin correto da curva com Dead
Time/Lag plausíveis; todos os caminhos de aborto reportam `stftCalAbortReason` correto; `learn()` do
LTFT comprovadamente congelado durante o procedimento.

---

## 4. Arquitetura v2 — controlador (Etapa 1)

### 4.1. Dois algoritmos selecionáveis (`correctionAlgorithm`)

```
correctionAlgorithm = 0  →  "Legacy Integrator"      (código atual, intocado, bit-a-bit)
correctionAlgorithm = 1  →  "Periodic Step" (novo)   (MoTeC-style, dead-time aware)
```

`0` é o valor que tunes antigas terão após migração → **comportamento idêntico ao atual sem nenhuma
ação do usuário** (zero-safe, ver seção 7). Tunes novas (`setDefaultStftSettings`) também nascem em
`0`; o modo novo é opt-in documentado.

### 4.2. Modo "Periodic Step"

A cada fast callback (200 Hz), por banco:

```
período_ms = interpolate2d(airflowEstimate, correctionPeriodFlowBins, correctionPeriodMs)
período_ms = clampF(100, período_ms, 5000)          // nunca mais rápido que 100 ms

se stftLearningState == enabled:
    acumula erro λ:  errSum += cell.getLambdaError(); errCount++

se timer_do_banco >= período_ms  E  errCount >= 10:  // ≥50 ms de amostras válidas
    avgError = errSum / errCount
    se |avgError| >= deadband:
        delta = clampF(-maxStepPercent, trimStepGain * avgError, +maxStepPercent)
        cell.applyStep(delta)                        // clampado em maxAdd/maxRemove existentes
    zera acumulador; timer_do_banco.reset()
```

- **Média do erro** na janela em vez de amostra instantânea → imune a ruído do sensor (a v1 usava
  amostra única).
- **Mudança de região** (`stftCorrectionBinIdx`) zera acumulador e timer — a primeira correção na
  célula nova espera um período completo (o λ observado ainda reflete a célula anterior).
- **DFCO/accel/fuel-cut**: `getLearningState()` já pausa a acumulação; se ao expirar o período houver
  menos de 10 amostras, o passo é adiado até completar.
- `cell.applyStep(delta)` é um método novo em `ClosedLoopFuelCellBase`: `m_adjustment = clampF(min,
  m_adjustment + delta, max)` — reusa os clamps `maxAdd`/`maxRemove` existentes.

**Por que converge rápido sem oscilar**: o plant λ↔combustível tem ganho ≈ −1 (adicionar x % de
combustível baixa λ ≈ x %). Como cada passo só acontece depois que o efeito do passo anterior está
**totalmente visível** no sensor (1 período = t95), com `ganho = g` o erro restante após n passos é
≈ `(1−g)ⁿ`. Com g = 0,5 (default) e período de 400 ms em cruzeiro: erro de 10 % → <1,5 % em ~1,2 s.
O integrador legado com τ = 30 s leva 30 s para 63 %. **~25× mais rápido, com margem de estabilidade
estrutural** (nunca corrige mais rápido do que enxerga).

### 4.3. Interação com LTFT

O LTFT continua integrando em direção ao resultado do STFT a cada 5 ms (`learn()`), o que funciona
igual nos dois modos — o output `clResult.banks[]` continua contínuo, só o *update* interno do STFT
que é discreto. Única mudança: freeze durante calibração (item 4 da auditoria).

---

## 5. Arquitetura v2 — auto-calibrador (Etapa 2)

### 5.1. Máquina de estados

```
Idle → Arming → WaitingStable → StepApplied(gravando buffer) → Settling → Analyze → Done → Idle
                                              ↓ (qualquer condição de aborto)
                                           Aborted → Idle
```

Diferença estrutural vs v1: **nenhuma decisão é tomada com amostra instantânea**. Durante o degrau,
λ é gravado num buffer estático e a extração de Dead Time/Lag/Período é feita **retrospectivamente**
sobre a curva completa, depois que ela assentou.

### 5.2. Medição retrospectiva

- Buffer estático (arquivo `stft_calibrator.cpp`, só banco 0): `static uint16_t buf[512]` — λ×10⁴
  amostrado a 100 Hz (a cada 2 fast callbacks) = janela de 5,12 s, **1 KB de RAM estática**.
- Baseline = média dos 500 ms da janela de estabilidade pré-degrau (a mesma que arma o degrau).
- Degrau aplicado: `trim = 1 + stepPercent/100` multiplicado em `result.banks[0]` (STFT e LTFT
  congelados).
- **Assentamento** detectado quando a média móvel de 300 ms varia < 0,005 λ por 500 ms, ou timeout
  de 4,5 s (aborta se Δλ observado < 50 % do esperado — sensor não respondeu).
- Análise sobre o buffer com média móvel de 5 amostras:
  - `Δ = settled − baseline` (delta **real**, não o previsto — a v1 usava o previsto)
  - `t10` = primeiro cruzamento de 10 % de Δ → **Dead Time**
  - `t90` = primeiro cruzamento de 90 % de Δ
  - `Lag = (t90 − t10) / 2,197` (primeira ordem: t90−t10 = ln(9)·τ) — equivale à fórmula MoTeC
    `(Period − DeadTime)/3` com t95, mas t10/t90 são muito mais robustos a ruído que 5 %/95 %.
  - `Período = DeadTime + 3×Lag` (t95 reconstruído da regressão, não do cruzamento cru)
- Sanidade: `DeadTime ∈ [20, 2000] ms`, `Lag ∈ [20, 1500] ms`, `Δ` com o sinal esperado. Fora disso
  → `Aborted`.

### 5.3. Segurança (pré-condições de armamento + aborto contínuo)

Armamento (`Arming`) exige **todas**:
- `stftCorrectionState == stftEnabled` e `getLearningState(bank0) == stftEnabled`
- Região atual ∈ {Idle, Cruise} — **bloqueado em Power** (degrau pobre sob carga = detonação/EGT)
  e em Overrun (perto de DFCO, λ instável)
- `|λ − target| < 0,03` (o loop já convergiu; o STFT congelado não vai derivar durante a medição)
- Estabilidade por 500 ms: RPM ±5 %, carga ±10 %, sem TPS accel recente
  (`TpsAccelEnrichment::getTimeSinceAcell() > 1 s`)

Aborto contínuo (qualquer estado ativo):
- Sensor λ inválido, λ > 1,25 ou < 0,7
- RPM desviou > ±10 % ou carga > ±15 % do ponto armado
- TPS accel, DFCO, qualquer fuel cut, região mudou, motor parou
- Comando `STFT_STOP_CALIBRATION`, timeout global de 8 s
- `stepPercent` sempre clampado em `[−15, −5]` (degrau pobre; enriquecedor não é oferecido porque
  Power está bloqueado)

### 5.4. Aplicação do resultado (1×, na transição para `Done`)

```cpp
void ShortTermFuelTrim::applyCalibrationResult(const StftCalResult& r) {
    auto& cfg = engineConfiguration->stft;
    // 1. Grava o Período no bin de vazão mais próximo do ponto atual
    size_t idx = nearestBin(engine->engineState.airflowEstimate, cfg.correctionPeriodFlowBins);
    cfg.correctionPeriodMs[idx] = clampF(100, r.periodMs, 5000);   // scaled_channel: valor de engenharia

    // 2. Para quem continua no modo legado: sugere timeConstant seguro (3× o período)
    size_t region = stftCorrectionBinIdx;
    cfg.cellCfgs[region].timeConstant = clampF(0.5f, 3.0f * r.periodMs / 1000.0f, 100.0f);

    efiPrintf("STFT cal @ %.0f kg/h: dead=%.0fms lag=%.0fms period=%.0fms -> bin[%d], TC[region %d]=%.1fs",
              ...);
}
```

Mesmo padrão do ETB autocal: firmware escreve na config, usuário faz "Read from ECU" + Burn no
TunerStudio para persistir. Os três valores medidos ficam no live data até a próxima calibração.

### 5.5. Ciclo de vida e persistência (decisões explícitas)

- **Início: manual** (botão no TS). O degrau forçado é intrusivo — o operador precisa estar
  segurando um ponto estável. Um clique = um ponto da curva (bin de vazão mais próximo); repetir em
  outros pontos de operação. Calibração contínua/oportunista em background (medição passiva usando
  transientes naturais de target, sem degrau) fica como possível fase futura — é outra feature.
- **Parada: sempre automática.** Sucesso (assentamento detectado, 1–5 s típico), aborto por qualquer
  guarda da seção 5.3 (teto absoluto: timeout de 8 s) ou botão Stop. O degrau nunca sobrevive ao
  timeout.
- **Flash: não grava automaticamente.** Resultado vai para a config em RAM; usuário persiste com
  "Read from ECU" + Burn (padrão ETB autocal). Alternativa avaliada e adiada: chamar
  `setNeedToWriteConfiguration()` — o storage manager só executa a gravação com motor parado
  (`storage.cpp:46-52`; no F405 escrever flash congela a CPU, então o burn é adiado de qualquer
  forma), mas o tune aberto no TS ficaria dessincronizado e um Burn posterior sem "Read from ECU"
  sobrescreveria a calibração. Se virar opção, deve ser opt-in.

### 5.6. Trigger

`bench_mode_e` (append no **final** do enum em `firmware/controllers/algo/engine_types.h`, hoje
termina em `LUA_COMMAND_10 = 42`):

```cpp
    STFT_START_CALIBRATION, // 43
    STFT_STOP_CALIBRATION,  // 44
```

Handler em `bench_test.cpp` junto aos cases LTFT (`bench_test.cpp:425`):

```cpp
case STFT_START_CALIBRATION: requestStftCalibration(true);  return;
case STFT_STOP_CALIBRATION:  requestStftCalibration(false); return;
```

`requestStftCalibration()` só seta um flag; `getCorrection()` (fast callback) consome. O comando roda
na thread do TS — nada de tocar a máquina de estados fora do fast callback.

---

## 6. Mudanças por arquivo

### 6.1. `firmware/integration/rusefi_config.txt` — **Etapa 1** (exceto `stftCalStepPercent`: Etapa 2)

```diff
 #define STFT_CELL_COUNT 4
+#define STFT_PERIOD_CURVE_SIZE 8
+
+#define stft_algo_e_enum "Legacy Integrator", "Periodic Step (dead-time aware)"
```

Append no **final** de `stft_s` (após `cellCfgs`), arrays primeiro para alinhamento natural:

```diff
     stft_cell_cfg_s[STFT_CELL_COUNT iterate] cellCfgs;
+
+    uint16_t[STFT_PERIOD_CURVE_SIZE] autoscale correctionPeriodFlowBins;Airflow axis for the correction period curve;"kg/h", 0.1, 0, 0, 6000, 1
+    uint16_t[STFT_PERIOD_CURVE_SIZE] correctionPeriodMs;Full lambda response period (dead time + 3x lag) at this airflow.\nThe periodic-step STFT waits this long between corrections. Use auto-calibration to populate.;"ms", 1, 0, 0, 5000, 0
+
+    custom stft_algo_e 1 bits, U08, @OFFSET@, [0:0], @@stft_algo_e_enum@@
+    stft_algo_e correctionAlgorithm;Legacy: continuous integrator with per-region time constant.\nPeriodic Step: applies gain*error once per full sensor response period - faster and delay-aware.
+    int8_t stftCalStepPercent;Fuel step applied during auto-calibration. Negative = lean step (safe at idle/cruise).;"%", 1, 0, -15, -5, 0
+    uint8_t autoscale trimStepGain;Fraction of the lambda error corrected per step. 0.5 = correct half the error each period.;"ratio", 0.01, 0, 0.1, 1, 2
+    uint8_t autoscale maxStepPercent;Maximum trim change per correction step.;"%", 0.1, 0, 0.5, 5, 1
 end_struct
```

Na entrega por etapas, `stftCalStepPercent` sai deste diff e entra na Etapa 2 como novo append ao
final de `stft_s` (mesma mecânica). 36 bytes a mais em `stft_s` no total → tudo depois de `stft` em
`engine_configuration_s` desloca. Isso é
rotina no rusEFI (assinatura muda, TS migra por nome de campo), mas **conferir que o tamanho total
da config não estoura o limite da página do board** — `gen_config_board.sh avonik_a1` acusa (o
Makefile roda isso automaticamente; nesta máquina não há toolchain, então validar via CI).

### 6.2. `firmware/controllers/math/short_term_fuel_trim_state.txt` (live data) — `stftPeriodMs`: **Etapa 1**; canais `stftCal*`: **Etapa 2**

```diff
     float[FT_BANK_COUNT iterate] stftInputError;STFT: input Lambda error; "%", 100, 0, 50, 150, 1
+
+    uint16_t stftPeriodMs;STFT: active correction period from curve; "ms", 1, 0, 0, 5000, 0
+    uint8_t stftCalState;STFT cal state (0 Idle 1 Arming 2 WaitStable 3 Step 4 Settling 5 Done 6 Aborted); "", 1, 0, 0, 6, 0
+    uint8_t stftCalAbortReason;STFT cal abort reason code; "", 1, 0, 0, 20, 0
+    uint16_t stftCalDeadTimeMs;STFT cal: measured dead time; "ms", 1, 0, 0, 5000, 0
+    uint16_t stftCalLagMs;STFT cal: measured sensor lag; "ms", 1, 0, 0, 5000, 0
+    uint16_t stftCalPeriodMs;STFT cal: computed period (dead + 3x lag); "ms", 1, 0, 0, 5000, 0
 end_struct
```

(`stftCalAbortReason` não existia na v1 — sem ele, todo aborto vira adivinhação no campo.)

### 6.3. [NEW] `firmware/controllers/math/stft_calibrator.{h,cpp}` — **Etapa 2**

Máquina de estados da seção 5. Esqueleto do header:

```cpp
#pragma once
#include "rusefi/timer.h"

enum class StftCalState : uint8_t { Idle, Arming, WaitingStable, StepApplied, Settling, Done, Aborted };
enum class StftCalAbort : uint8_t { None, SensorInvalid, LambdaRange, RpmDrift, LoadDrift,
                                    TpsTransient, FuelCut, RegionChange, RegionNotAllowed,
                                    Timeout, NoResponse, ImplausibleResult, UserStop, EngineStop };

struct StftCalResult {
    float deadTimeMs, lagMs, periodMs;
    bool valid;
};

class StftAutoCalibrator {
public:
    void requestStart(float stepPercent);   // chamado do fast callback ao consumir o flag do bench cmd
    void requestStop();
    // Roda a 200 Hz. Retorna o multiplicador de trim do degrau (1.0 fora de StepApplied/Settling).
    float update(float lambda, float lambdaTarget, float rpm, float load);
    bool isActive() const;                  // Arming..Settling
    bool consumeDoneEdge();                 // true exatamente 1x na transicao para Done
    StftCalState getState() const;
    StftCalAbort getAbortReason() const;
    const StftCalResult& getResult() const;
private:
    void abort(StftCalAbort reason);
    void analyzeBuffer();                   // t10/t90 -> dead/lag/period (secao 5.2)
    // estado armado: rpm/carga de referencia, baseline, timers, indice do buffer...
};
```

Todo o estado é membro/estático — **zero alocação dinâmica**. O buffer de 1 KB é `static` no `.cpp`
(instância única, só banco 0, coerente com a v1).

### 6.4. `firmware/controllers/math/closed_loop_fuel_cell.{h,cpp}` — **Etapa 1**

```diff
 	// Get the current adjustment amount, without altering internal state.
 	float getAdjustment() const;
+
+	// Periodic-step mode: apply one discrete correction step (clamped to min/max adjustment).
+	void applyStep(float delta);
```

`update()` (modo legado) fica **intocado**.

### 6.5. `firmware/controllers/math/closed_loop_fuel.{h,cpp}` — despacho/passo periódico: **Etapa 1**; itens do calibrador: **Etapa 2**

- Membros novos: `StftAutoCalibrator m_calibrator;`, por banco `Timer m_stepTimer[FT_BANK_COUNT]`,
  `float m_errSum[FT_BANK_COUNT]`, `uint16_t m_errCount[FT_BANK_COUNT]`, `ft_region_e m_lastRegion`.
- `getCorrection()`:
  1. Consome o flag de request do bench command → `m_calibrator.requestStart(clamp do stftCalStepPercent)`.
  2. Se `stftCorrectionState != stftEnabled` → aborta calibração ativa (como na v1) e retorna.
  3. Roda `m_calibrator.update(...)` → `calTrim` (1.0 quando inativo).
  4. **Se calibrando**: congela update/step das células; `result.banks[0] = cell.getAdjustment() * calTrim`.
  5. Senão: despacha por `correctionAlgorithm` — legado chama `cell.update()` como hoje (linha 149 atual
     intocada); periodic step roda a lógica da seção 4.2.
  6. `if (m_calibrator.consumeDoneEdge()) applyCalibrationResult(...)` — **uma vez**.
  7. Atualiza live data (`stftPeriodMs`, `stftCal*`).
- Novo público: `bool isCalibrationActive() const { return m_calibrator.isActive(); }`
- Novo livre: `void requestStftCalibration(bool start);` (flag consumido no fast callback).

### 6.6. `firmware/controllers/long_term_fuel_trim.cpp` — **Etapa 2**

```diff
 	if ((!cfg.enabled) || (ltftSavePending) || (ltftLoadPending) ||
+		(engine->module<ShortTermFuelTrim>()->isCalibrationActive()) ||
 		(engine->module<ShortTermFuelTrim>()->stftCorrectionState != stftEnabled)) {
```

### 6.7. `firmware/controllers/algo/engine_types.h` + `firmware/controllers/bench_test.cpp` — **Etapa 2**

Seção 5.6 (dois valores no final de `bench_mode_e` + dois cases no `executeTSCommand`).

### 6.8. `firmware/controllers/math/math.mk` — **Etapa 2**

`+ $(CONTROLLERS_MATH)/stft_calibrator.cpp`

### 6.9. `firmware/controllers/algo/defaults/default_fuel.cpp` — **Etapa 1** (`stftCalStepPercent`: Etapa 2)

`setDefaultStftSettings()` — manter os defaults atuais das células (o τ=30 s continua correto para o
modo legado, que segue sendo o default) e adicionar:

```cpp
	// Correction period curve - generic NA engine, sensor ~40-60 cm downstream.
	// Physics: period ~ exhaust volume / volumetric flow + sensor response.
	static const float flowBins[STFT_PERIOD_CURVE_SIZE] = { 5, 10, 20, 40, 80, 120, 200, 350 }; // kg/h
	static const float periods[STFT_PERIOD_CURVE_SIZE]  = { 900, 650, 450, 320, 240, 200, 160, 130 }; // ms
	copyArray(cfg.correctionPeriodFlowBins, flowBins);
	copyArray(cfg.correctionPeriodMs, periods);

	cfg.correctionAlgorithm = static_cast<stft_algo_e>(0);  // legacy - modo novo e opt-in
	cfg.stftCalStepPercent = -10;
	cfg.trimStepGain = 0.5f;
	cfg.maxStepPercent = 2.0f;
```

### 6.10. `firmware/controllers/algo/defaults/default_base_engine.cpp` (`defaultsOrFixOnBurn`) — **Etapa 1** (linha do `stftCalStepPercent`: Etapa 2)

Migração de tunes antigas (todos os campos novos chegam = 0) conforme `docs/calibration-compatibility.md`:

```cpp
	if (engineConfiguration->stft.correctionPeriodFlowBins[STFT_PERIOD_CURVE_SIZE - 1] == 0) {
		// curva nunca inicializada -> defaults
		(mesma curva default da secao 6.9)
	}
	if (engineConfiguration->stft.stftCalStepPercent == 0) engineConfiguration->stft.stftCalStepPercent = -10;
	if (engineConfiguration->stft.trimStepGain == 0)   engineConfiguration->stft.trimStepGain = 0.5f;
	if (engineConfiguration->stft.maxStepPercent == 0) engineConfiguration->stft.maxStepPercent = 2.0f;
	// correctionAlgorithm == 0 e intencionalmente valido: Legacy (comportamento atual)
```

### 6.11. `firmware/tunerstudio/tunerstudio.template.ini` — itens 1 e 3 (curva/algoritmo): **Etapa 1**; itens 2 e dialog de cal: **Etapa 2**

1. **Curva** (nomes gerados dos campos: conferir no `.ini` gerado — membros de `stft_s` saem
   prefixados, ex. `stft_correctionPeriodFlowBins`):

```ini
	curve = stftPeriodCurve, "STFT Correction Period"
		columnLabel = "Airflow", "Period"
		xAxis = 0, 400, 10
		yAxis = 0, 1500, 10
		xBins = stft_correctionPeriodFlowBins, mafEstimate
		yBins = stft_correctionPeriodMs
		showTextValues = true
```

   (`mafEstimate` é o canal de saída existente em kg/h — `output_channels.txt:256` — o cursor da
   curva segue o ponto de operação ao vivo.)

2. **Comandos** (junto aos `cmd_ltft_*`, `tunerstudio.template.ini:2194`):

```ini
cmd_stft_start_cal = "@@TS_IO_TEST_COMMAND_char@@@@ts_command_e_TS_BENCH_CATEGORY_16_hex@@@@bench_mode_e_STFT_START_CALIBRATION_16_hex@@"
cmd_stft_stop_cal  = "@@TS_IO_TEST_COMMAND_char@@@@ts_command_e_TS_BENCH_CATEGORY_16_hex@@@@bench_mode_e_STFT_STOP_CALIBRATION_16_hex@@"
```

3. **Dialog** no `fuelClosedLoopDialog`: seletor `stft_correctionAlgorithm`, `stft_trimStepGain`,
   `stft_maxStepPercent`, painel da curva, e painel de calibração com:

```ini
	dialog = stftAutoCalDialog, "Auto-Calibration"
		field = "Warm engine, steady idle or cruise. A lean step (-5..-15%) runs for 1-5 s."
		field = "Fuel step", stft_stftCalStepPercent
		commandButton = "Start calibration", cmd_stft_start_cal, { fuelClosedLoopCorrectionEnabled == 1 }
		commandButton = "Stop / abort",      cmd_stft_stop_cal
```

   Resultados/estado ao vivo: **gauges** (`stftCalState`, `stftCalDeadTimeMs`, `stftCalLagMs`,
   `stftCalPeriodMs`, `stftCalAbortReason` numa `gaugeCategory` "STFT Calibration") — dialogs do TS
   não mostram output channels como `field`; a v1 errava nisso. Mensagens detalhadas saem no console
   (`efiPrintf`).

### 6.12. Fork-specific (este repositório) — **ambas as etapas**

- Board ativa é `avonik_a1`; o `make` do board regenera headers + `.ini` (não commitar gerados).
- Web-tuner: após o build de firmware que muda a assinatura, rodar `npm run archive-ini` (fluxo de
  versionamento de ini do web-tuner) — sem isso o auto-load por assinatura não encontra o ini novo.
- Nesta máquina não há toolchain (make/gcc/arm-none-eabi) — build e unit tests validam via CI/WSL.

---

## 7. Compatibilidade de calibração

| Campo novo | Valor em tune antiga | Efeito | Mitigação |
|---|---|---|---|
| `correctionAlgorithm` | 0 | Modo legado — **comportamento idêntico ao atual** | Nenhuma necessária (zero-safe por construção) |
| `correctionPeriod*` | 0 | Só usado no modo novo; curva zerada com clamp mínimo de 100 ms seria degenerada | `defaultsOrFixOnBurn()` popula a curva default |
| `stftCalStepPercent` | 0 | Calibração sem degrau (no-op) | `defaultsOrFixOnBurn()` → −10 |
| `trimStepGain` / `maxStepPercent` | 0 | Passo zero (no-op) | `defaultsOrFixOnBurn()` → 0,5 / 2,0 % |

Campos novos ficam no **final** de `stft_s`; o deslocamento do restante de `engine_configuration_s`
é absorvido pela migração por nome do TS + bump de assinatura (rotina do projeto).

---

## 8. Testes (`unit_tests/tests/test_stft.cpp` + novo `test_stft_calibrator.cpp`)

Infra existente confirmada: `EngineTestHelper`, `advanceTimeUs(...)`, `engine->rpmCalculator.setRpmValue(...)`,
`Sensor::setMockValue(...)`, macro `ITERATE_TIME` (ver `test_long_term_fuel_trim.cpp`). Para
`getCorrection()` retornar `stftEnabled` o teste precisa: `fuelClosedLoopCorrectionEnabled = true`,
`startupDelay = 0`, mock de CLT ≥ `minClt`, `setRpmValue(...)` e mock de `Lambda1`;
`engine->fuelComputer.targetLambda` é atribuível direto.

**Planta simulada** (a melhoria mais importante vs v1 — valida a medição, não só a máquina de estados):

```cpp
// First-order + dead time: lambda(t) = l0 + delta * (1 - exp(-(t - theta)/tau)) para t > theta
struct FopdtLambdaPlant { float l0, delta, thetaS, tauS; ... };
```

| Teste | Etapa | O que garante |
|---|---|---|
| `PeriodicStep_WaitsFullPeriod` | 1 | Nenhum passo antes do período expirar; timer novo (estado inicial do `Timer` = "expirado") dá 1º passo imediato |
| `PeriodicStep_StepIsGainTimesAvgError` | 1 | `delta = ganho × média(erro)`, clamp de `maxStepPercent` respeitado |
| `PeriodicStep_ConvergesIn3Periods` | 1 | Com planta FOPDT, erro de 10 % → <2 % em 3 períodos, **sem overshoot além do deadband** |
| `PeriodicStep_RegionChangeResetsWindow` | 1 | Troca de bin zera acumulador/timer |
| `Legacy_BitForBitRegression` | 1 | `correctionAlgorithm=0` reproduz exatamente o comportamento atual (testes existentes `ClosedLoopCell.*`/`ClosedLoopFuel.*` passam sem alteração) |
| `Calibrator_MeasuresFopdtPlant` | 2 | θ=150 ms, τ=100 ms na planta → dead time medido ±20 ms, lag ±15 % |
| `Calibrator_AbortsOnNoise/NoResponse/Timeout/RpmDrift` | 2 | Cada guarda de aborto, com `stftCalAbortReason` correto |
| `Calibrator_RefusesPowerRegion` | 2 | Não arma em Power |
| `Calibrator_AppliesResultOnce` | 2 | `consumeDoneEdge()` dispara 1× ; curva e timeConstant com **valores de engenharia** corretos (pega o bug de dupla escala) |
| `Ltft_FrozenDuringCalibration` | 2 | `learn()` não integra com calibração ativa |

A planta FOPDT é escrita na Etapa 1 (o teste de convergência já a usa) e reutilizada nos testes do
calibrador na Etapa 2.

```bash
cd unit_tests && ./test.sh   # suite completa; CI nos 4 toolchains (sem toolchain local nesta máquina)
```

---

## 9. Verificação em bancada/campo

**Etapa 1:**

1. **Regressão**: tune antiga carregada → `correctionAlgorithm = Legacy`, STFT se comporta como antes.
2. **Curva**: TS → Fuel → Short Term Fuel Trim → curva "STFT Correction Period" com cursor seguindo
   `mafEstimate` ao vivo. Preencher manualmente: em cada ponto estável, degrau no alvo de λ e medir
   no log o tempo até assentar.
3. **Modo novo**: mudar para "Periodic Step", Burn; com deadband 0,5 % observar no log: correção em
   degraus discretos espaçados pelo período, convergência < 2 s após perturbação de VE de 10 %, sem
   oscilação sustentada. Se oscilar: período da curva está curto para o ponto (aumentar/recalibrar)
   ou ganho alto (baixar para 0,3).

**Etapa 2:**

4. **Calibração** (motor quente, marcha lenta estável): Start → gauges mostram
   `Arming → WaitingStable → Step → Settling → Done` e Dead Time/Lag/Period preenchidos; console
   mostra o resumo; "Read from ECU" traz a curva atualizada; Burn persiste. Repetir em cruzeiro leve
   (~2 500 rpm) e moderado — 3 pontos ancoram a curva, o resto interpola. Comparar com os valores
   medidos manualmente na Etapa 1 (devem bater dentro de ~±20 %).
5. **LTFT**: `ltftLearning == false` durante toda a calibração.

---

## 10. Resumo de arquivos (por etapa)

| Arquivo | Etapa | Mudança |
|---|---|---|
| `firmware/integration/rusefi_config.txt` | 1 (+2) | E1: `STFT_PERIOD_CURVE_SIZE`, curva 8 pts (kg/h→ms), `correctionAlgorithm`, ganho/passo máx. E2: `stftCalStepPercent` (int8) |
| `firmware/controllers/math/short_term_fuel_trim_state.txt` | 1 (+2) | E1: `stftPeriodMs`. E2: +5 canais de calibração (estado, motivo de aborto, dead/lag/period) |
| `firmware/controllers/math/closed_loop_fuel_cell.{h,cpp}` | 1 | +`applyStep()`; `update()` intocado |
| `firmware/controllers/math/closed_loop_fuel.{h,cpp}` | 1 (+2) | E1: despacho por algoritmo + passo periódico. E2: integração do calibrador, `isCalibrationActive()` |
| `firmware/controllers/algo/defaults/default_fuel.cpp` | 1 (+2) | E1: defaults da curva/ganhos (legado permanece default). E2: default do step de cal |
| `firmware/controllers/algo/defaults/default_base_engine.cpp` | 1 (+2) | Migração zero-safe em `defaultsOrFixOnBurn()` |
| `firmware/tunerstudio/tunerstudio.template.ini` | 1 (+2) | E1: curva + seletor/ganhos no dialog. E2: dialog de cal, 2 commandButtons, gaugeCategory |
| **[NEW]** `firmware/controllers/math/stft_calibrator.{h,cpp}` | 2 | Máquina de estados + buffer 1 KB + análise retrospectiva t10/t90 |
| `firmware/controllers/long_term_fuel_trim.cpp` | 2 | Freeze do learn durante calibração |
| `firmware/controllers/algo/engine_types.h` | 2 | +`STFT_START_CALIBRATION`, +`STFT_STOP_CALIBRATION` (final do `bench_mode_e`) |
| `firmware/controllers/bench_test.cpp` | 2 | 2 cases novos |
| `firmware/controllers/math/math.mk` | 2 | +`stft_calibrator.cpp` |
| `unit_tests/tests/test_stft.cpp` + **[NEW]** `test_stft_calibrator.cpp` | 1 (+2) | E1: 5 testes (passo periódico + regressão) e planta FOPDT. E2: 5 testes do calibrador |

---

## 11. Fontes

- MoTeC — [GPRP Pro Setup & Tuning Guide (PDF)](https://www.motec.com.au/hessian/uploads/GPRP_Pro_Setup_and_Tuning_Guide_967042122e.pdf): Dead Time + Period (95 %), procedimento de degrau
- MoTeC Forum — [Lambda period compensation](https://forum.motec.com.au/viewtopic.php?f=53&t=5202&p=23721): tabela de delay vs Exhaust Mass Flow
- MoTeC Forum — [Closed loop fueling](https://forum.motec.com.au/viewtopic.php?f=66&t=5404) / [Closed loop lambda control](https://forum.motec.com.au/viewtopic.php?f=11&t=725)
- BimmerBlog — [Fuel Closed Loop Period and Exhaust Gas Speed on MoTeC M1](https://bimmerblog.org/2025/07/04/fuel-closed-loop-period-and-exhaust-gas-speed-on-motec-m1/)
- Emtron — [Lambda Transport Delay Guide](https://help.emtronaustralia.com.au/emtune/Newtopic466.html): delay vs Air Mass Flow (g/s) compensando um PID
- Link ECU Forums — [Advice setting Closed Loop Lambda (G4X)](https://forums.linkecu.com/topic/11954-advice-setting-closed-loop-lambda/): gain table + update rate table
- HP Academy — [Webinar 086: Closed Loop Fuel Control Link G4+](https://www.hpacademy.com/forum/webinar-questions/show/086-closed-loop-fuel-control-link-g4/)
- MSExtra — [Let's talk EGO delay tuning](https://www.msextra.com/forums/viewtopic.php?p=571852) e [EGO control PID rpm dependency](https://www.msextra.com/forums/viewtopic.php?f=131&t=66523); [MS3-Pro manual, Basic EGO settings](https://www.manualslib.com/manual/1331880/Megasquirt-Ms3-Pro.html?page=151)
- SAE — [2007-01-1342: An Adaptive Delay-Compensated PID Air Fuel Ratio Controller](https://www.sae.org/publications/technical-papers/content/2007-01-1342/)
- ScienceDirect — [Disturbance rejection control of air–fuel ratio with transport-delay in engines](https://www.sciencedirect.com/science/article/abs/pii/S0967066118301588)
- Patente US7987840 — [Delay compensated air/fuel control of an IC engine](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7987840)
- rusEFI Forum — [Closed-loop fuel control (short & long term)](https://www.rusefi.com/forum/viewtopic.php?f=5&t=1046)
