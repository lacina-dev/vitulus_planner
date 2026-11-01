#!/usr/bin/env python3
"""Řetěz programů v planneru (majitel 2026-10-31, mise #277).

Skutečný node_planner bez ROS masteru (stuby a dočasný HOME z test_site_programs).

Run:  python3 test/test_program_chain.py
"""
import json
import os
import shutil
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_site_programs as S  # noqa: E402
from test_site_programs import String, last  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

CHAIN = '/web_plan/program_chain'
STATE = '/web_plan/program_chain_state'
ACTIVE = '/web_plan/program_active'
UNFINISHED = '/web_plan/program_active_unfinished'


@unittest.skipIf(S._SKIP is not None, S._SKIP)
class ProgramChainTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        S._load_module()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(S._HOME, ignore_errors=True)

    def setUp(self):
        shutil.rmtree(os.path.join(S._HOME, '.vitulus'), ignore_errors=True)
        for site in ('A', 'B'):
            S.make_site(site)
        del S.EVENTS[:]
        del S.LOGS[:]
        S.GRID['grid'] = S.grid(400)
        S.GRID['fail'] = False
        S.STATUS['site'] = None
        n = S.new_node()
        S.serve(n, 'A')
        n.callback_map_save_zone(S.zmsg('Near', 2, 2, 6, 6))
        n.callback_map_save_zone(S.zmsg('Far', 12, 12, 17, 17))
        n.callback_program_new(S.program('ZoneA', S.zone_msgs(n, 'Near')))
        n.callback_program_new(S.program('Test', S.zone_msgs(n, 'Far')))
        n.callback_program_new(S.program('Third', S.zone_msgs(n, 'Near', 'Far')))
        self.n = n
        self.yaml_before = S.yaml_programs('A')
        del S.EVENTS[:]

    def sent(self, topic):
        return [m for t, m in S.EVENTS if t == topic]

    def state(self):
        return json.loads(last(STATE).data)

    def test_chain_goes_as_one_start(self):
        self.n.callback_program_select(String('ZoneA>>>Test>>>Third'))
        first = self.sent(ACTIVE)
        self.assertEqual([p.name for p in first], ['ZoneA'])
        chain = self.sent(CHAIN)
        self.assertEqual(len(chain), 1)
        self.assertEqual([p.name for p in chain[0].program_list], ['ZoneA', 'Test', 'Third'])
        self.assertTrue(first[0].header.frame_id.startswith('chain:'))
        self.assertEqual(first[0].header.frame_id, chain[0].header.frame_id)
        # the chain is published before the program
        order = [t for t, _m in S.EVENTS if t in (CHAIN, ACTIVE)]
        self.assertEqual(order, [CHAIN, ACTIVE])
        self.assertTrue(chain[0].program_list[1].zone_list[0].paths)   # zones with paths
        self.assertEqual(self.state()['programs'], ['ZoneA', 'Test', 'Third'])
        self.assertEqual(self.state()['skipped'], [])

    def test_programs_stay_untouched(self):
        self.n.callback_program_select(String('ZoneA>>>Test'))
        self.assertEqual(S.mem_programs(self.n), {'ZoneA': ['Near'], 'Test': ['Far'],
                                                  'Third': ['Near', 'Far']})
        self.assertEqual(S.yaml_programs('A'), self.yaml_before)
        for program in self.n.permanent_program_list_msg.program_list:
            self.assertEqual(program.header.frame_id, '')

    def test_plain_run_sends_no_chain(self):
        self.n.callback_program_select(String('ZoneA'))
        self.assertEqual(len(self.sent(ACTIVE)), 1)
        self.assertEqual(self.sent(ACTIVE)[0].header.frame_id, '')
        self.assertEqual(self.sent(CHAIN), [])
        self.assertEqual(self.sent(STATE), [])

    def test_program_of_another_map_is_left_out_with_a_reason(self):
        self.n.callback_program_select(String('ZoneA>>>Elsewhere>>>Test'))
        self.assertEqual([p.name for p in self.sent(CHAIN)[0].program_list], ['ZoneA', 'Test'])
        skipped = self.state()['skipped']
        self.assertEqual([(row['name'], row['code']) for row in skipped],
                         [('Elsewhere', 'not_on_map')])
        self.assertIn('dok', skipped[0]['text_cs'])
        self.assertTrue(any('Elsewhere' in m.data for m in self.sent('/web_plan/log')))

    def test_nothing_left_runs_the_first_alone(self):
        self.n.callback_program_select(String('ZoneA>>>Elsewhere'))
        self.assertEqual(self.sent(CHAIN), [])
        self.assertEqual(self.sent(ACTIVE)[0].name, 'ZoneA')
        self.assertEqual(self.sent(ACTIVE)[0].header.frame_id, '')
        self.assertEqual(self.state()['programs'], ['ZoneA'])

    def test_first_unknown_starts_nothing(self):
        self.n.callback_program_select(String('Elsewhere>>>ZoneA'))
        self.assertEqual(self.sent(ACTIVE), [])
        self.assertEqual(self.sent(CHAIN), [])

    def test_twice_runs_once(self):
        self.n.callback_program_select(String('ZoneA>>>Test>>>ZoneA'))
        self.assertEqual([p.name for p in self.sent(CHAIN)[0].program_list], ['ZoneA', 'Test'])
        self.assertEqual(self.state()['skipped'][0]['code'], 'twice')

    def test_program_that_cannot_run_is_left_out(self):
        for program in self.n.permanent_program_list_msg.program_list:
            if program.name == 'Test':
                program.zone_list = []
        self.n.callback_program_select(String('ZoneA>>>Test>>>Third'))
        self.assertEqual([p.name for p in self.sent(CHAIN)[0].program_list], ['ZoneA', 'Third'])
        self.assertEqual(self.state()['skipped'][0]['code'], 'unusable')

    def test_resume_resumes_the_first_only(self):
        for program in self.n.permanent_program_list_msg.program_list:
            program.last_result = 'failed: on_path-Near-0-0'
        self.n.callback_program_select_resume(String('ZoneA>>>Test'))
        self.assertEqual(self.sent(ACTIVE), [])
        first = self.sent(UNFINISHED)
        self.assertEqual(first[0].name, 'ZoneA')
        self.assertEqual(first[0].last_result, 'failed: on_path-Near-0-0')
        self.assertTrue(first[0].header.frame_id.startswith('chain:'))
        self.assertEqual(self.state()['verb'], 'resume')

    def test_site_prefix(self):
        self.n.callback_program_select(String('ZoneA>>>A|||Test>>>B|||Third'))
        self.assertEqual([p.name for p in self.sent(CHAIN)[0].program_list], ['ZoneA', 'Test'])
        self.assertEqual(self.state()['skipped'][0]['name'], 'Third')

    def test_run_path_follows_the_running_program(self):
        # /web_plan/program_marker(_state): the map shows the program that runs.
        n = self.n
        n.callback_program_select(String('ZoneA>>>Test'))
        n.callback_run_path_busy(Bool(True))
        n.callback_mission_status(String('Initiating'))
        n.callback_mission_program(String('ZoneA'))
        self.assertEqual(json.loads(last('/web_plan/program_marker_state').data)['run'], 'ZoneA')
        n.callback_mission_status(String('Initiating'))     # NextProgramState
        n.callback_mission_program(String('Test'))
        state = json.loads(last('/web_plan/program_marker_state').data)
        self.assertEqual(state['run'], 'Test')
        self.assertTrue(n._program_is_running('Test'))

    def test_a_program_really_named_so_is_a_program(self):
        self.n.callback_program_new(S.program('A>>>B', S.zone_msgs(self.n, 'Near')))
        del S.EVENTS[:]
        self.n.callback_program_select(String('A>>>B'))
        self.assertEqual(self.sent(ACTIVE)[0].name, 'A>>>B')
        self.assertEqual(self.sent(CHAIN), [])


if __name__ == '__main__':
    unittest.main()
