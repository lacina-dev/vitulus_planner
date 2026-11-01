"""Several saved programs, one run (owner 2026-10-31, mission #277).

The program ref on /web_plan/program_select[_resume] may name a chain:
    'ZoneA>>>Test'    ZoneA, then Test, docking only after Test
Same topic, same message as a single program; a Run from the UI and the
calendar never sends it. Resume resumes the first program, the others run from
their start. The programs stay untouched, a chain is never saved.

The first program goes the normal way (`_select_program`), so it gets every
check a single program gets. The moment it is handed to the mission the rest is
resolved against the map that is on: a program that cannot run in this chain is
left out and the reason goes on /web_plan/program_chain_state, the others run.
The robot does not change maps on the way (that takes the dock).

The mission gets all programs with their current zones on /web_plan/program_chain
(see mower_unit_smach/chain.py), so nothing is asked at a program boundary.

No ROS here except the messages: `ChainStart` runs against an `env`.
"""
import json

from std_msgs.msg import String
from vitulus_msgs.msg import PlannerProgramList

SEP = '>>>'
SITE_SEP = '|||'
MARK = 'chain:'

# (en, cs): what the owner hears about a left-out program.
REASONS = {
    'not_on_map': ("program '{name}' is not on the map that is on, it would need the dock",
                   "program {name} není na zapnuté mapě, potřeboval by dok"),
    'unusable': ("program '{name}' cannot run: {detail}",
                 "program {name} nejde spustit: {detail}"),
    'twice': ("program '{name}' is twice in the chain, it runs once",
              "program {name} je v řetězu dvakrát, pojede jednou"),
}


def parse(text):
    """Refs of a program select, in order."""
    return [part.strip() for part in (text or '').split(SEP) if part.strip()]


def split_ref(ref):
    """(site or None, name) of 'SITE|||Name' or 'Name'."""
    site, sep, name = ref.rpartition(SITE_SEP)
    return (site or None), name


def resolve(first, refs, env):
    """(programs after `first` that run, [left-out dicts])."""
    taken = [first.name]
    programs, skipped = [], []

    def leave(name, code, **params):
        en, cs = REASONS[code]
        skipped.append({'name': name, 'code': code,
                        'text_en': en.format(name=name, **params),
                        'text_cs': cs.format(name=name, **params)})

    for ref in refs:
        site, name = split_ref(ref)
        if name in taken:
            leave(name, 'twice')
            continue
        listed = env.chain_program(name, site)
        if listed is None:
            leave(name, 'not_on_map')
            continue
        program, reason = env.chain_current(listed)
        if program is None:
            leave(name, 'unusable', detail=reason)
            continue
        taken.append(name)
        programs.append(program)
    return programs, skipped


class ChainStart(object):
    """Stands in for the publisher `_select_program` hands the program to."""

    def __init__(self, env, refs, verb, pub):
        self.env = env
        self.refs = refs
        self.verb = verb
        self.pub = pub

    def publish(self, first):
        programs, skipped = resolve(first, self.refs[1:], self.env)
        chain_id = MARK + self.env.chain_id()
        if programs:
            first.header.frame_id = chain_id
            chain = PlannerProgramList(program_list=[first] + programs)
            chain.header.frame_id = chain_id
            self.env.chain_pub.publish(chain)
        names = [first.name] + [p.name for p in programs]
        for row in skipped:
            self.env.chain_log('Chain: ' + row['text_en'])
        notes = self.env.start_notes() if hasattr(self.env, 'start_notes') else []
        self.env.chain_state_pub.publish(String(json.dumps(
            {'id': chain_id, 'verb': self.verb, 'requested': self.refs, 'programs': names,
             'skipped': skipped, 'notes': notes})))
        self.pub.publish(first)
