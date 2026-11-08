"""Je teď dok povolen?  Master se ptá uzlu `dock_permit` (T18, etapa 3).

Povolení doku drží robotí vrstva: uzel `dock_permit` (balík `vitulus_dock`,
kontrakt `predavka/t18-kontrakt.md`) publikuje latched `/dock_permit/state`
(při změně a každých 10 s).  Master ho odebírá a ptá se jen tady — robot tak
na agentovi nezávisí (dřívější most přes `vitulus_agent.v2.ros_wire` odpadl).

    on_state(json_text)  -> None      krmí odběr /dock_permit/state
    allowed(now=None)    -> (ok, why, state)
    homeward()           -> dict | None   konec okna s robotem venku (D38)
    spend / give_back    -> True      no-op: výjezdy počítá uzel sám
    intent_supported()   -> bool       umí uzel značku výjezdu (Q1)?
    INTENT.request / result            značka bez čekání (FSM mastra)
    exit_intent(reason)  -> (ok, why)  totéž s čekáním (nástroje, integrace)

Q1 (owner 8. 11.): do „jen 1 výjezd“ / `max_exits` se počítají jen výjezdy
agenta.  Master proto před startem běhu z rady agenta pošle uzlu značku
`/dock_permit/exit_intent` (req_id, platnost `INTENT_TTL_S`) a vyjede, až ji
uzel potvrdí na `/dock_permit/result` (vyhodnotí v dalším kroku FSM, nečeká).
Jen když uzel značku umí (stav obsahuje klíč `exit_intent`); starý uzel
počítá každý výjezd a master jede bez značky jako dřív.  Kalendář značku
neposílá, jeho výjezd uzel nezapočte.

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
import uuid

STATE_TOPIC = '/dock_permit/state'
INTENT_TOPIC = '/dock_permit/exit_intent'
RESULT_TOPIC = '/dock_permit/result'
#: Q1: značka čeká na výjezd nejvýš tolik s; když se robot mezitím vydá
#: z doku (manévr), platí až do výjezdu (nejvýš 30 min).  Uzel bere 10..600 s.
INTENT_TTL_S = 120.0
#: FSM: odpověď uzlu se čte v dalším kroku; dřív než po INTENT_WAIT_S se
#: chybějící odpověď nepočítá za neúspěch.  Potvrzená značka se použije, jen
#: když není starší než INTENT_USE_S (pak by mohla do výjezdu vypršet).
#: 8 s: respawn uzlu (5 s + start) nesmí radu zahodit; 3 neúspěchy ≈ 25–30 s.
INTENT_WAIT_S = 8.0
INTENT_USE_S = 60.0
#: Odpověď, která nepřišla do tolika s, se za neúspěch počítá; delší mezera
#: mezi požadavkem a dalším pohledem = přerušení (STOP, konec okna), ne uzel.
INTENT_ANSWER_S = 20.0
#: Absolutní strop čekání běhu na značku (i když se přerušení nepočítají):
#: 30 min od první značky nebo 20 odeslaných značek → intent_timeout.
INTENT_MAX_WAIT_S = 1800.0
INTENT_MAX_SENT = 20
#: Po tolika nepotvrzených pokusech master běh rady ukončí (záskok kalendáře).
INTENT_MAX_FAILS = 3
#: Blokující varianta (exit_intent, nástroje): 2 s na odpověď, 2 pokusy.
INTENT_TIMEOUT_S = 2.0
INTENT_ATTEMPTS = 2
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

    def intent_supported(self, t=None):
        """Umí uzel značku výjezdu (Q1)?  Nový uzel má ve stavu klíč
        `exit_intent` (i s hodnotou null); starý ne."""
        doc = self._fresh(t)[0]
        return isinstance(doc, dict) and 'exit_intent' in doc

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


class IntentLink(object):
    """Q1: značka výjezdu agenta pro uzel a čekání na jeho potvrzení.

    `setup(publish)` dostane funkci, která pošle JSON text na
    `/dock_permit/exit_intent`; `on_result(text)` krmí odběr
    `/dock_permit/result`.  `send()` vrátí (ok, why) a nikdy nevyhazuje.
    Bez potvrzení master nevyjede (jinak by se výjezd agenta nezapočetl)."""

    def __init__(self):
        self._publish = None
        self._cond = threading.Condition()
        self._got = {}

    def setup(self, publish):
        self._publish = publish

    def on_result(self, data):
        try:
            doc = json.loads(data) if isinstance(data, str) else data
        except (TypeError, ValueError):
            return
        if not isinstance(doc, dict) or doc.get('cmd') != 'exit_intent' or \
                doc.get('req_id') is None:
            return
        with self._cond:
            self._got[str(doc['req_id'])] = doc
            while len(self._got) > 50:
                self._got.pop(next(iter(self._got)))
            self._cond.notify_all()

    def request(self, reason='', by='master', ttl_s=None, rid=None):
        """Pošli značku a hned se vrať.  → req_id, nebo None (bez spojení /
        chyba).  Odpověď později `result(req_id)`."""
        try:
            if self._publish is None:
                return None
            rid = rid or 'master-%s' % uuid.uuid4().hex[:12]
            ttl_s = INTENT_TTL_S if ttl_s is None else ttl_s
            self._publish(json.dumps({'by': by, 'req_id': rid, 'ttl_s': float(ttl_s),
                                      'reason': str(reason or '')[:200]}, sort_keys=True))
            return rid
        except Exception:                               # noqa: BLE001
            return None

    def result(self, rid):
        """Odpověď uzlu na značku `rid` (dict, jednou), nebo None."""
        with self._cond:
            return self._got.pop(str(rid), None) if rid is not None else None

    def send(self, reason='', by='master', ttl_s=None, timeout=None, attempts=None):
        """Blokující varianta (nástroje, integrace; FSM mastra ne)."""
        ttl_s = INTENT_TTL_S if ttl_s is None else ttl_s
        timeout = INTENT_TIMEOUT_S if timeout is None else timeout
        attempts = INTENT_ATTEMPTS if attempts is None else attempts
        try:
            if self._publish is None:
                return False, 'no_link'
            rid = 'master-%s' % uuid.uuid4().hex[:12]
            for _ in range(max(1, int(attempts))):
                self.request(reason, by, ttl_s, rid=rid)
                end = time.monotonic() + float(timeout)
                with self._cond:
                    while rid not in self._got:
                        left = end - time.monotonic()
                        if left <= 0:
                            break
                        self._cond.wait(left)
                    got = self._got.pop(rid, None)
                if got is not None:
                    return bool(got.get('ok')), str(got.get('code') or '')
            return False, 'intent_timeout'
        except Exception as exc:                        # noqa: BLE001
            return False, 'intent_error: %s' % exc


VIEW = PermitView()
INTENT = IntentLink()


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


def intent_supported():
    return VIEW.intent_supported()


def exit_intent(reason=''):
    """Q1: řekni uzlu, že další výjezd z doku je agentův, a počkej na
    potvrzení.  → (ok, why).  Bez potvrzení nevyjíždět."""
    return INTENT.send(reason=reason)


def give_back(release):
    """No-op (T18): nepovedený výjezd uzel nezapočte."""
    return True
