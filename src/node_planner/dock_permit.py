"""Je teď dok povolen?  Master se ptá uzlu `dock_permit` (T18, etapa 3).

Povolení doku drží robotí vrstva: uzel `dock_permit` (balík `vitulus_dock`,
kontrakt `predavka/t18-kontrakt.md`) publikuje latched `/dock_permit/state`
(při změně a každých 10 s).  Master ho odebírá a ptá se jen tady — robot tak
na agentovi nezávisí (dřívější most přes `vitulus_agent.v2.ros_wire` odpadl).

    on_state(json_text)  -> None      krmí odběr /dock_permit/state
    allowed(now=None)    -> (ok, why, state)
    homeward()           -> dict | None   konec okna s robotem venku (D38)
    spend / give_back    -> True      no-op: výjezdy počítá uzel sám

Fail-closed: bez stavu, se stavem starším než `FRESH_S` nebo nečitelným je
„nepovoleno".  Čte se JEN před výjezdem z doku (K1): jízdu, sečení ani návrat
robota, který je venku, `allowed` nikdy neblokuje.  Kalendář se neptá (D37).

Q2 (owner rozhodne): na který běh se `homeward` uplatní, určuje jediné místo —
parametr mastra `~dock_permit_homeward` (výchozí `HOMEWARD_FOR`).
"""
import json
import os
import threading
import time

STATE_TOPIC = '/dock_permit/state'
#: Kontrakt §8: stav chybí nebo je starší než 30 s = nepovoleno.
FRESH_S = 30.0
#: Q2 výchozí: homeward jen pro běh, který spustila rada agenta.
#: 'agent' | 'all' (i kalendář) | 'none' (nikdy, robot dojede jako dřív).
HOMEWARD_FOR = 'agent'
HOMEWARD_CHOICES = ('agent', 'all', 'none')
#: D38: dosekat rozdělaný díl nejvýš ~5 min (uzel posílá finish_chunk_s).
FINISH_CHUNK_S = 300.0
#: S8: kdo smí povolit dok — JEDINÝ zdroj pravdy, týž soubor čte uzel
#: (`~owners_file`) i agent.  Jména jen na robotu, nikdy v repu.  Bez souboru
#: (prázdný, nečitelný) master žádné okno za povolení nebere (fail-closed).
OWNERS_FILE = os.path.expanduser('~/.vitulus/dock_permit_owners')


def read_owners(path=None):
    """(jména, stav) — stejná pravidla jako uzel: jedno jméno na řádek,
    `#` komentář, bez ohledu na velikost písmen.  stav: ok | missing |
    empty | unreadable.  Nikdy nevyhazuje."""
    path = path or OWNERS_FILE
    try:
        if not os.path.exists(path):
            return [], 'missing'
        with open(path, encoding='utf-8-sig') as fh:   # BOM ≠ část jména
            names = [ln.split('#', 1)[0].strip().lower() for ln in fh]
    except Exception:                                   # noqa: BLE001
        return [], 'unreadable'
    names = [n for n in names if n]
    return (names, 'ok') if names else ([], 'empty')


class PermitView(object):
    """Poslední `/dock_permit/state` a jeho čerstvost (monotonní čas příjmu)."""

    def __init__(self, fresh_s=FRESH_S, mono=time.monotonic):
        self.fresh_s = float(fresh_s)
        self.mono = mono
        self._lock = threading.Lock()
        self._doc = None
        self._rx = None

    def on_state(self, data, t=None):
        try:
            doc = json.loads(data) if isinstance(data, str) else data
        except (TypeError, ValueError):
            return
        if not isinstance(doc, dict) or 'allowed' not in doc:
            return
        with self._lock:
            self._doc = doc
            self._rx = self.mono() if t is None else float(t)

    def _fresh(self, t=None):
        with self._lock:
            doc, rx = self._doc, self._rx
        if doc is None:
            return None, None, 'no_state'
        age = (self.mono() if t is None else float(t)) - rx
        if age > self.fresh_s or age < -1.0:
            return None, age, 'stale'
        return doc, age, None

    def state(self, t=None):
        return self._fresh(t)[0]

    def allowed(self, t=None, owners=None):
        """S7: povolení je jen otevřené okno podepsané ownerem ze souboru
        ownerů; why `no_owners_<stav>` / `not_owner`, když ne."""
        try:
            doc, _age, bad = self._fresh(t)
            if doc is None:
                return False, bad, None
            ok = doc.get('allowed') is True
            if not ok:
                return False, str(doc.get('why') or 'locked'), doc
            names, status = owners if owners is not None else read_owners()
            if status != 'ok':
                return False, 'no_owners_' + status, doc
            by = str((doc.get('active') or {}).get('by') or '').strip().lower()
            if by not in names:
                return False, 'not_owner', doc
            return True, str(doc.get('why') or 'open'), doc
        except Exception:                               # noqa: BLE001
            return False, 'unreadable', None

    def homeward(self, t=None):
        """Latched `homeward` uzlu, s `left_s` do konce dosekání (na tomto
        stroji: deadline − stamp stavu − doba od příjmu), nebo None."""
        try:
            doc, age, _bad = self._fresh(t)
            hw = (doc or {}).get('homeward')
            if not isinstance(hw, dict):
                return None
            chunk = float(hw.get('finish_chunk_s') or FINISH_CHUNK_S)
            left = chunk
            if hw.get('deadline') is not None and doc.get('stamp') is not None:
                left = float(hw['deadline']) - float(doc['stamp']) - max(0.0, age)
            out = dict(hw)
            out['left_s'] = max(0.0, min(chunk, left))
            out['key'] = '%s@%s' % (hw.get('window'), hw.get('since'))
            return out
        except Exception:                               # noqa: BLE001
            return None


VIEW = PermitView()


def on_state(data):
    VIEW.on_state(data)


def allowed(now=None):
    """(ok, why, state) — smí robot TEĎ vyjet z doku?  Nic nespotřebuje."""
    return VIEW.allowed()


def homeward():
    return VIEW.homeward()


def fresh():
    """Je stav uzlu čerstvý?  (Chybějící/starý stav ≠ „homeward zrušen".)"""
    return VIEW.state() is not None


def homeward_for(value):
    """Platná volba Q2 (neznámá hodnota = výchozí)."""
    value = str(value or '').strip().lower()
    return value if value in HOMEWARD_CHOICES else HOMEWARD_FOR


def spend(release):
    """No-op (T18): výjezd započte uzel, až robot dok opustí."""
    return True


def give_back(release):
    """No-op (T18): nepovedený výjezd uzel nezapočte."""
    return True
