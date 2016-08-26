from __future__ import print_function
import os
import re
import sys
import yaml
import kconfiglib

from subprocess import call
from shutil import copyfile
from jinja2 import Environment, FileSystemLoader

def multiple_replace(dict, text):
    "common function to replace multiple patterns in string"
    regex = re.compile("(%s)" % "|".join(map(re.escape, dict.keys())))

    # For each match, look-up corresponding value in dictionary
    return regex.sub(lambda mo: dict[mo.string[mo.start():mo.end()]], text)

def removekey(d, key):
    "remove a key from a dictionary and return potentially smaller dictionary"
    if key in d:
        r = dict(d)
        del r[key]
        return r
    else:
        return d

def depend(opts, dep):
    "recursive dependency function expects a dictionary of lists "
    if len(opts) == 0:
        return opts

    lst = set()
    for o in opts:
        if o in dep:
            for d in dep[o]:
                print(o)
                lst.add(d)
                dep = removekey(dep, o)

    return opts | depend(lst, dep)

class Generator:

    def __init__(self, board, target, args):
        self.board = board
        self.target = target
        self.settings = {}      # values for templates
        self.configs = {}       # for kconfig configs
        self.paths = []         # list of files

        # Need a few paths to move files around
        tools_path = os.path.dirname(os.path.realpath(__file__))
        self.nuttx_path = os.path.dirname(tools_path)
        self.base_path = os.path.dirname(self.nuttx_path)
        confs_path = os.path.join(self.nuttx_path, "configs")

        self.template_path = os.path.join(tools_path, "templates")
        self.board_path = os.path.join(confs_path, self.board)

        kconfs_path = os.path.join(tools_path, "kconfs.yaml")
        # read the configuration file
        with open(kconfs_path) as y:
            self.kconfs = yaml.safe_load(y)

        # setup options
        # format for dependencies is:
        # {'a': ['b', 'c'], 'b': ['c', 'e'], 'e': ['d'], 'f': []}
        # where an option has a list of dependencies
        dependencies = {}       # configuration dependencies
        for k in self.kconfs.keys():
            if 'dependencies' in self.kconfs[k] and self.kconfs[k]['dependencies'] is not None:
                dependencies[k] = self.kconfs[k]['dependencies']
            else:
                dependencies[k] = []
        options = depend(set(args), dependencies)

        # setup settings, configs and paths from selected options
        for o in options:
            print('config_' + o)
            self.settings['config_' + o] = True
            try:
                for k in self.kconfs[o]['configs']:
                    self.configs[k] = self.kconfs[o]['configs'][k]
            except TypeError: pass

            if o in self.kconfs and self.kconfs[o]['files'] is not None:
                self.paths = self.paths + self.kconfs[o]['files']


    def path_to_rel_template(self, path):
        d = { '$board' : 'board', '$target' : 'target', }
        return multiple_replace(d, path)

    def path_to_abs_target(self, path):
        "return absolute path of target"
        d = { '$board' : self.board, '$target' : self.target, }
        p = multiple_replace(d, path)
        return os.path.join(self.base_path, p)

    def update_files(self):
        "copies and processes the required template files"
        env = Environment(loader = FileSystemLoader(self.template_path))
        for sub_path in self.paths:
            sub_tmpl = self.path_to_rel_template(sub_path)
            sub_tgt = self.path_to_abs_target(sub_path)

            print(sub_tmpl, " -> ", sub_tgt)
            template = env.get_template(sub_tmpl)
            tgt_path = sub_tgt
            tgt_dir = os.path.dirname(tgt_path)
            if not os.path.exists(tgt_dir):
                os.makedirs(tgt_dir)
            with open(tgt_path, 'w') as f:
                f.write(template.render(self.settings, board=self.board, target=self.target))

    def update_defconfig(self):
        "Applies the configuration settings to the defconfig"
        os.environ['APPSDIR'] = '../apps' # TODO: fixme

        # load the Kconfig tree
        conf = kconfiglib.Config(os.path.join(self.nuttx_path, 'Kconfig'), base_dir=self.nuttx_path)

        # load our default defconfig
        conf.load_config(self.path_to_abs_target('nuttx/configs/$board/$target/defconfig'))
        for k in self.configs:
            # allow for variable names in the values of config entries
            # MANIFEST=$board-$target is the only case at the time of
            # writing.
            d = { '$board' : self.board, '$target' : self.target, }
            v = multiple_replace(d, str(self.configs[k]))
            print(k, v)
            try:
                conf.get_symbol(k).set_user_value(v)
            except AttributeError:
                print(k, v)
        # write the output
        conf.write_config(self.path_to_abs_target('nuttx/configs/$board/$target/defconfig'))

def main(argv):
    "generate an mdk-tmplt/argv[1] configuration all additional arguments are settings"
    gen = Generator('mdk-tmplt', argv[1], argv[2:])
    gen.update_files()
    gen.update_defconfig()

def usage(argv):
    tools_path = os.path.dirname(os.path.realpath(__file__))
    kconfs_path = os.path.join(tools_path, "kconfs.yaml")
    with open(kconfs_path) as y:
        kconfs = yaml.safe_load(y)
        args = [k for k in kconfs.keys()]
        arg_str = (", ".join(args))
        print("usage: python {} <board target> [{}]\n".format(argv[0], arg_str))
        print("  at least one configuration from the set in [] must be selected")
        print("  The generated configuration will be installed under: ")
        print("     nuttx/configs/mdk-tmplt/<board target>/")
    

if __name__ == "__main__":
    if len(sys.argv) < 3: 
        usage(sys.argv)
        sys.exit(-1)

    main(sys.argv)
