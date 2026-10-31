# Bench test: spánek s vypnutým NUC (robot v doku)

Provádí Robert, s robotem v doku a bez běžícího programu. Kalendář ani přepínač automatiky se
nemění: testuje se jen power modul (PM) a BIOS NUCu. Nikdo jiný ten test nespouští.

**Co je potřeba zjistit**
1. Nabootuje NUC sám po návratu 19 V po *tvrdém* odříznutí?
2. Nabootuje sám i po *čistém* `poweroff`? To je rozhodující. BIOS NUC8 má „After Power Failure":
   Stay Off / Last State / Power On. S **Last State** NUC po čistém vypnutí zůstane vypnutý, po
   tvrdém řezu naskočí. Všech posledních 7 bootů v journalu skončilo tvrdým řezem, čistý
   poweroff tu nikdy neproběhl, takže zatím nevíme nic.
3. Jak dlouho trvá poweroff (musí se vejít do W = 180 s) a boot do plného stacku.

**Kdyby NUC nenaskočil:** do uplynutí času spánku tlačítko PM nezabere, firmware robota hned zase
vypne. Po uplynutí času PM pustí 19 V; pak stačí krátce stisknout tlačítko přímo na NUCu, nebo na
PM podržet 2–4 s (vypne) a znovu 2–4 s (zapne).

Příprava (robot v doku, supply ONLINE):

    source ~/catkin_ws/devel/setup.bash
    rostopic echo -n1 /pm/power_status | grep -E 'sleep_|supply_status|module_status'

## A. Tvrdý řez, 3 minuty

Podle UI (Settings › Power › Sleep control), nebo:

    rostopic pub -1 /pm/set_sleep_wait_before_standby std_msgs/UInt64 "data: 30000"
    rostopic pub -1 /pm/set_sleep_time std_msgs/UInt64 "data: 180000"
    rostopic echo -n1 /pm/power_status | grep -E 'sleep_wait_standby|sleep_time_interval'   # 30000 / 180000
    rostopic pub -1 /pm/set_robot_sleep std_msgs/Bool "data: true"
    rostopic echo -n1 /pm/power_status | grep -E '^sleep_status'                            # True

Čekáte: za 30 s 19 V pryč (NUC zhasne), za 3 min od příkazu 19 V zpět a NUC sám nabootuje.
Poznamenat čas příkazu a čas, kdy jde znovu přihlásit SSH.

## B. Čistý poweroff, 3 minuty (rozhodující)

    rostopic pub -1 /pm/set_sleep_wait_before_standby std_msgs/UInt64 "data: 120000"
    rostopic pub -1 /pm/set_sleep_time std_msgs/UInt64 "data: 180000"
    rostopic echo -n1 /pm/power_status | grep -E 'sleep_wait_standby|sleep_time_interval'   # 120000 / 180000
    rostopic pub -1 /pm/set_robot_sleep std_msgs/Bool "data: true"
    rostopic echo -n1 /pm/power_status | grep -E '^sleep_status'                            # True
    date; sudo systemctl poweroff

Čekáte: NUC se vypne sám do 2 min, v 2:00 PM sebere 19 V, ve 3:00 vrátí 19 V, NUC **sám** nabootuje.

- Naskočil: BIOS je v pořádku, nainstalovat sudoers (níže).
- Nenaskočil: BIOS: F2 při startu › Power › Secondary Power Settings › After Power Failure =
  **Power On** (název položky ověřit na místě). Pak zopakovat B. Bez toho sudoers neinstalovat:
  robot pak spí tvrdým řezem po `sync`, což funguje, ale není čisté.

Po startu změřit:

    journalctl -b -1 -n 40 --no-pager | tail -5     # kdy skončilo vypínání
    journalctl -b -1 --no-pager | grep -m1 -E 'Stopping Vitulus'
    systemd-analyze
    rostopic echo -n1 /scheduler/status

Doba od `Stopping Vitulus` po poslední řádek musí být výrazně pod 180 s; jinak zvětšit
`sleep_wait_before_standby_s` v `vitulus_planner/config/master_controller.yaml`.

## C. Po testu

    rostopic pub -1 /pm/set_sleep_time std_msgs/UInt64 "data: 28800000"    # původní hodnota 8 h

`sleep_wait_standby` nechat, master si ho nastaví sám na 180000 (zapíše se jen při změně).

Instalace oprávnění (až po úspěšném B):

    sudo install -m 0440 -o root -g root \
        ~/catkin_ws/src/vitulus/vitulus_planner/config/sudoers-vitulus-sleep /etc/sudoers.d/vitulus-sleep
    sudo visudo -cf /etc/sudoers.d/vitulus-sleep

## D. Nepovinné: drift časovače PM

Stejně jako A s `set_sleep_time` 28800000 (8 h) přes noc. Rozdíl od 8 h 00 min (počítáno od
příkazu, čas ze `systemd-analyze` / `last -x reboot`) dělený 8 h je drift; `sleep_timer_drift` v
yamlu má být aspoň tolik. Výchozí 0,5 % je odhad pro keramický rezonátor Arduina Mega.
