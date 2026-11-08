"""Je teď dok povolen?  Jediná otázka mastra na dokový zámek majitele.

Rozhodnutí majitele 2026-11-08: povolování doku zůstává na něm.  Rada agenta
„sekej" smí robota vyslat jen při platném povolení, stejně jako jiné výjezdy.
Zámek žije ve vrstvě agenta (vitulus_agent.v2.ros_wire: rozhodnutí z UI
v ~/.vitulus/agent/v2/work.db, záložně /etc/vitulus/dock_release.yaml).  Tady
je jen tenký most bez rospy, ať se master ptá jediné funkce:

    allowed(now=None)  -> (ok, why, release)   ros_wire.dock_permitted, nic nespotřebuje
    spend(release)     -> bool                 použití svolení těsně před startem
    give_back(release) -> bool                 program dok neopustil: použití zpět

Fail-closed: bez modulu zámku nebo s nečitelným zámkem dok povolen není.
Kalendář se zámku neptá (jeho sloty povolil majitel tím, že je naplánoval).
"""
ACTION = 'mower.program_run'


def _wire():
    from vitulus_agent.v2 import ros_wire
    return ros_wire


def allowed(now=None):
    try:
        return _wire().dock_permitted(ACTION, now=now)
    except Exception as exc:  # noqa: BLE001 - nečitelný zámek je zamčený
        return False, 'dock lock unavailable: %s' % exc, None


def spend(release):
    try:
        return bool(_wire().note_use(ACTION, release))
    except Exception:  # noqa: BLE001
        return False


def give_back(release):
    try:
        return bool(_wire().return_use(ACTION, release))
    except Exception:  # noqa: BLE001
        return False
