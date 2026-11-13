import json,math,re,unittest
from pathlib import Path
import refresh
class GeneratorTests(unittest.TestCase):
    def test_literal_safety(self):
        self.assertEqual(refresh.number('-12_in',{}),-12)
        self.assertEqual(refresh.number('SPEED/2',{'SPEED':100}),50)
        with self.assertRaises(ValueError):refresh.number('__import__("os").system("echo bad")',{})
    def test_comment_strings(self):
        self.assertIn('http://local',refresh.clean('"http://local" // comment'))
    def test_catalog(self):
        html=(Path(__file__).parent/'index.html').read_text()
        catalog=json.loads(re.search(r'const catalog=(.+);\nconst \$',html).group(1))
        self.assertEqual(len(catalog['autons']),18)
        exact=[a for a in catalog['autons'] if a['kind']=='Exact profile reference']
        self.assertEqual(len(exact),3)
        for route in exact:
            self.assertEqual((route['samples'][0]['x'],route['samples'][0]['y']),(0,0))
            self.assertEqual((route['samples'][-1]['x'],route['samples'][-1]['y']),(24,24))
            self.assertTrue(all(b['t']>a['t'] for a,b in zip(route['samples'],route['samples'][1:])))
        drive=next(a for a in catalog['autons'] if a['id']=='drive_example')
        self.assertAlmostEqual(drive['samples'][-1]['y'],0)
        for name in ['interfered_example','measure_offsets']:
            route=next(a for a in catalog['autons'] if a['id']==name)
            self.assertEqual(route['kind'],'Recorded run needed');self.assertEqual(route['samples'],[])
if __name__=='__main__':unittest.main()
