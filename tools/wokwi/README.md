# Banco STEP/DIR en Wokwi

El proyecto usa una Raspberry Pi Pico 2 (RP2350, Cortex-M33). Abre la raíz del
repositorio en Visual Studio Code. El [circuito](../../diagram.json) conecta un
analizador de ocho canales a estas señales.

| Canal | GPIO | Señal |
|---|---|---|
| D0 | GP2 | STEP 1 |
| D1 | GP3 | DIR 1 |
| D2 | GP4 | STEP 2 |
| D3 | GP5 | DIR 2 |
| D4 | GP6 | Marcador de ventana de prueba |
| D5–D7 | | Libres |

El firmware tiene cuatro salidas STEP/DIR consecutivas y carece de una salida
ENABLE. Para el banco, STEP está activo en alto y DIR positivo corresponde a
alto en ambos ejes. Las direcciones iniciales son negativa y positiva,
respectivamente. El PIO se configura a 1 MHz y el planificador a 100 kHz. Los
mínimos de STEP alto, STEP bajo, establecimiento y mantenimiento de DIR son
10 µs, tomados de la configuración de banco del proyecto.

## Fase 1 · Compatibilidad

La documentación de la Pico 2 anuncia un núcleo Cortex-M33, GPIO, DMA,
temporizadores y GDB. Las instrucciones PIO añadidas por el RP2350 y los núcleos
restantes quedan fuera del modelo. `step_dir.pio` usa `pull block`, `out`,
`jmp` e `irq` del conjunto original. La ejecución real también depende de
la FIFO TX unida, DREQ, el encadenamiento DMA, las IRQ del PIO y el indicador
persistente `TXSTALL`. Esos comportamientos requieren la captura de la fase 3.
La resolución del VCD y los tiempos simulados tienen alcances distintos de una
medición eléctrica sobre la placa.
[Referencia de la Pico 2](https://docs.wokwi.com/parts/board-pi-pico-2#simulation-features)

## Fases 2 y 3 · Compilar, simular y capturar

1. Instala **Wokwi Simulator** (`Wokwi.wokwi-vscode`) y activa su licencia con
   **Wokwi: Request a New License** si lo solicita la extensión.
2. Instala el target con `rustup target add thumbv8m.main-none-eabihf`.
3. Ejecuta **Tasks: Run Task** → **Wokwi: compilar Pico 2**. En la terminal,
   desde la raíz, el mismo build es:

   ```bash
   (cd firmware/pico2 && cargo build --release --locked --features wokwi-pio-test --target-dir target/wokwi)
   ```

4. Ejecuta **Wokwi: Start Simulator**. Deja correr al menos 1.5 s de tiempo
   simulado, el mismo presupuesto inicial del script de la CLI.
5. Pulsa **Stop** en la simulación. La extensión exporta `wokwi.vcd` en la
   raíz. Ábrelo con Surfer, GTKWave o PulseView y agrega D0 a D4. En PulseView
   selecciona VCD como formato de importación si te lo pide.
   [Configuración de Wokwi para VS Code](https://docs.wokwi.com/vscode/project-config)
   · [Guía del analizador lógico](https://docs.wokwi.com/guides/logic-analyzer)

La tarea compila el ELF ARM que utiliza `wokwi.toml`. Su directorio de salida es
distinto del build habitual de la placa. Ambos perfiles seleccionan la Pico 2,
reutilizan el planificador, la codificación, el ensamblado y la SM0 de PIO0.

| Perfil | Feature | Alimentación de PIO | Ventanas |
|---|---|---|---|
| `pio` | `wokwi-pio-test` | `CpuExecutor` del repositorio, TX de cuatro palabras | 10 |
| `dma` | `wokwi-test` | `StreamingDmaExecutor` habitual, TX unida y dos canales DMA | 11 |

El firmware prepara las palabras antes de abrir cada ventana y acciona GP6
desde la SM1, fuera de la ejecución crítica de la SM0. La salida GP6 solo se
configura en las builds de prueba. Si una ejecución devuelve un resultado
inesperado, el analizador informa la ventana sin cierre o las ventanas faltantes.

## Fases 4 y 5 · Verificar una captura

```bash
cargo test --locked -p scara-wokwi
cargo run --quiet --locked -p scara-wokwi -- compare wokwi.vcd
cargo run --quiet --locked -p scara-wokwi -- inspect wokwi.vcd
```

`compare` comprueba las ventanas de GP6, los niveles iniciales, los cambios
STEP/DIR, su orden, la anchura, ambos desplazamientos y la ausencia de pasos
fuera de una prueba. Cada ventana se alinea por su primer cambio STEP/DIR, lo que
separa la latencia de inicialización de los intervalos de movimiento. La
simultaneidad se comprueba dentro de un cuanto del VCD. Para el resto de los
instantes relativos, el margen inicial es un ciclo de PIO (1000 ns) más un
cuanto del VCD. Para comprobar los mínimos en Wokwi, se admite un ciclo de
`clk_sys` por cada extremo del intervalo medido (13.334 ns en total a 150 MHz),
más un cuanto del VCD. La captura de Wokwi mostró variaciones de 6 a 13 ns en
intervalos de STEP de 10 µs. Un VCD con resolución más gruesa que el ciclo de
PIO falla la comparación.
Los tiempos se conservan en femtosegundos enteros. `inspect` muestra los
periodos, frecuencias y tiempos de STEP y DIR por pulso.

Puedes indicar nombres de señal con `--signals STEP1,DIR1,STEP2,DIR2,MARCADOR` y
polaridades con `--step-active high,low --dir-positive high,low`. Puedes cambiar
el margen temporal de `compare` con `--tolerance-ns N` según la resolución
conocida de la captura. Usa `--profile dma` si compilaste con la feature
`wokwi-test`. Cualquier discrepancia devuelve un código distinto de cero.

| Ventana | Caso | Pulsos por eje |
|---|---|---|
| 1 | Reposo, ejecución de END | 0, 0 |
| 2 | Solo primer motor | 3, 0 |
| 3 | Solo segundo motor, dirección negativa | 0, 3 |
| 4 | Dos motores, con dos flancos STEP simultáneos | 6, 2 |
| 5 | Ida y vuelta, inversión de DIR y frontera de segmento | 6, 0 |
| 6 | Aceleración, crucero y desaceleración | 16, 0 |
| 7 | Recta cartesiana de (1, 1) a (1.25, 0.75) | 7, 2 |
| 8 | Un solo pulso | 1, 0 |
| 9 | Pulsos de 10 µs en alto y en bajo, 50 kHz | 140, 140 |
| 10 | Espera de 50 ms entre dos pulsos | 2, 0 |
| 11 | Agotamiento inducido de FIFO tras dos bloques, perfil DMA | 128, 128 |

Los casos articulares usan velocidades de prueba entre 12.5 y 25 rad/s y
aceleración de 2500 rad/s² para acortar la simulación. La base de tiempo y los
mínimos de señal son los mismos que en el banco existente.

La recta usa cinemática inversa mediante `JointTrajectory`, dos brazos
unitarios de prueba y 32 pasos/rad. El analizador reconstruye la posición
cartesiana con cinemática directa y contrasta el error con la cota de
cuantización. El banco usa esos parámetros como geometría de prueba.

Para agregar un caso, incorpora un elemento a `CASES` en [src/lib.rs](src/lib.rs)
y genera sus órdenes con las APIs del proyecto. Agrega el conteo y el
desplazamiento calculados por separado. Las pruebas incluyen una captura
manual, una comparación de todas las ventanas con una traza sintética y la
derivación independiente de los tres pasos del primer eje.

## Fase 6 · CLI, CI y GDB

Instala [Wokwi CLI](https://docs.wokwi.com/wokwi-ci/cli-installation) y crea o
copia un **API token** activo en el [panel de Wokwi CI](https://wokwi.com/dashboard/ci).
Debe empezar con `wok_` y tener 44 caracteres. La licencia de VS Code pertenece
a un flujo distinto. En la shell donde ejecutarás la prueba:

```bash
read -rsp 'Pega el token de Wokwi CI y pulsa Enter ' WOKWI_CLI_TOKEN
printf '\n'
export WOKWI_CLI_TOKEN
bash tools/wokwi/run.sh
```

Si guardaste el token con permisos 600 en `~/.config/wokwi/cli.env`, ejecuta
`source "$HOME/.config/wokwi/cli.env"` antes del script. El script compila,
ejecuta Wokwi con `--vcd-file` y `--timeout`, y compara
`target/wokwi/step-dir-pio.vcd`. Puedes usar `WOKWI_TIMEOUT_MS` para aumentar la
duración simulada si quedan ventanas incompletas y `WOKWI_CLI` para indicar
otra ruta al ejecutable. Se permite que la CLI termine por tiempo porque
el comparador exige todas las ventanas del perfil. `WOKWI_PROFILE=dma` compila
el banco original y escribe `target/wokwi/step-dir-dma.vcd`; sirve para repetir
el diagnóstico de compatibilidad. En GitHub Actions, basta
con instalar la CLI y entregar `WOKWI_CLI_TOKEN` desde un secreto al mismo
script. Las pruebas normales de Cargo funcionan sin esta variable.
[CLI de Wokwi](https://docs.wokwi.com/wokwi-ci/cli-usage)
· [Escenarios YAML de Wokwi](https://docs.wokwi.com/wokwi-ci/automation-scenarios)

Para depurar, puedes descomentar `gdbServerPort` en `wokwi.toml` y usar
**Wokwi: Start Simulator and Wait for Debugger**. Con la CLI, arranca con
`wokwi-cli . --gdb-server-port 3333 --timeout 0`. En otra terminal:

```bash
arm-none-eabi-gdb firmware/pico2/target/wokwi/thumbv8m.main-none-eabihf/release/pico2-step-dir \
  -ex 'target remote localhost:3333'
```

Puedes usar `break main` y `continue`. Mantén el puerto comentado para analizar
las señales en una ejecución continua.

### Estado de la verificación local

- El linter de Wokwi CLI 0.29.1 aceptó el diagrama y sus pines.
- La feature de captura se compiló para la Pico 2. Los fixtures y los casos
  locales verifican el lector, las referencias y el comparador. Las 1370
  pruebas del workspace pasaron en debug y release.
- Con un token válido, Wokwi CLI produjo capturas VCD a 100 ms, 500 ms y 16 s
  simulados. Los dos STEP se mantuvieron en bajo y DIR2 subió a los 73.640 µs.
  La captura de 16 s registró cero ventanas completas. GDB ubicó el núcleo
  en `DmaPairTransfer::stop`, esperando que se limpie `DMA.CHAN_ABORT`
  durante la recuperación del primer END. El registro virtual leía
  `0xffffffff`. El banco DMA completo requiere
  resolver ese comportamiento en el simulador.
- En la prueba por CPU, GDB observó `PreloadFailed`. Wokwi informó TX llena con
  cuatro palabras pese a tener activo `FJOIN_TX`. Con la FIFO TX separada,
  el VCD mostró 10 µs de establecimiento de DIR y un STEP de 10 µs. El canal
  conectado a TP5 permaneció bajo en esa prueba. El marcador definitivo usa
  GP6 y comandos propios en la SM1.
- La ejecución final por CPU en Wokwi CLI 0.29.1, durante 1.5 s simulados,
  exportó `target/wokwi/step-dir-pio.vcd` y pasó las diez ventanas con una
  desviación máxima de 14 ns frente a las palabras de referencia. El caso de
  50 kHz registró 140 pulsos por eje y el cartesiano registró desplazamientos
  de −7 y −2 pasos.
- Al repetir el perfil DMA con el marcador final, la captura de 250 ms cerró
  la ventana de reposo y dejó abierta la del primer movimiento a los
  12.980 ms, sin flancos STEP. El script devolvió código 1.
