

from __future__ import print_function

import ast

class ReconfigureParser:

    """
    ReconfigureParser

    Simple class to parse messages from the ros_reconfigure package
    """

    def parse_config_desc(self, msg):
        res = []
        for group in msg.groups:
            newG = {'id'     : group.id,
                    'name'   : group.name,
                    'parent' : group.parent,
                    'type'   : group.type}
            newG['parameters'] = [self.parse_parameter(p, msg) for p in group.parameters]
            res.append(newG)
        return res

    def parse_parameter(self, param, configDesc):
        res = {'name'        : param.name,
               'type'        : param.type,
               'level'       : param.level,
               'description' : param.description,
               'edit_method' : self.parse_edit_method(param.edit_method)}
        res.update(self.parse_limits(param.name, param.type, configDesc))
        return res

    def parse_limits(self, pname, ptype, configDesc):
        res = {}
        if ptype == 'bool':
            res['min']     = self.find_limit_value(pname, configDesc.min.bools)
            res['max']     = self.find_limit_value(pname, configDesc.max.bools)
            res['default'] = self.find_limit_value(pname, configDesc.dflt.bools)
        elif ptype == 'int':
            res['min']     = self.find_limit_value(pname, configDesc.min.ints)
            res['max']     = self.find_limit_value(pname, configDesc.max.ints)
            res['default'] = self.find_limit_value(pname, configDesc.dflt.ints)
        elif(ptype == 'double'):
            res['min']     = self.find_limit_value(pname, configDesc.min.doubles)
            res['max']     = self.find_limit_value(pname, configDesc.max.doubles)
            res['default'] = self.find_limit_value(pname, configDesc.dflt.doubles)
        elif(ptype == 'group'):
            res['min']     = self.find_limit_value(pname, configDesc.min.groups)
            res['max']     = self.find_limit_value(pname, configDesc.max.groups)
            res['default'] = self.find_limit_value(pname, configDesc.dflt.groups)
        elif(ptype == 'str'):
            res['min']     = self.find_limit_value(pname, configDesc.min.strs)
            res['max']     = self.find_limit_value(pname, configDesc.max.strs)
            res['default'] = self.find_limit_value(pname, configDesc.dflt.strs)
        return res
        
    def find_limit_value(self, pname, limitList):
        tmp = next((n for n in limitList if n.name == pname),None)
        if tmp is not None:
            return tmp.value
        return None
    
    def parse_edit_method(self, method):
        res = {'type' : 'None'}
        if len(method) == 0:
            return res
        parsed = ast.literal_eval(method)
        if not 'enum' in parsed.keys():
            return res
        
        res['type']        = 'enum'
        res['description'] = parsed['enum_description']
        res['enum_values'] = [{'name'        : v['name'],
                               'description' : v['description'],
                               'type'        : v['type'],
                               'value'       : v['value']} for v in parsed['enum']]
        return res



