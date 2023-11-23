import os
import sys
import importlib.util
import importlib.machinery
import csv
import json

software_dir = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

def create_software_module():
    software_spec = importlib.util.spec_from_file_location('software', os.path.join(software_dir, '__init__.py'))
    software_module = importlib.util.module_from_spec(software_spec)

    software_spec.loader.exec_module(software_module)

    sys.modules['software'] = software_module

if 'software' not in sys.modules:
    create_software_module()

from software import util
from collections import OrderedDict

def escape(s):
    return json.dumps(s)

value_id_enum = []
value_id_list = []
value_id_infos = []
value_id_order = []
value_id_tree = OrderedDict()

groupings = [
    ('L1', 'L2', 'L3'),
    ('L1 N', 'L2 N', 'L3 N'),
    ('L1 L2', 'L2 L3', 'L3 L1'),
]

translation_values = {'en': [], 'de': []}
translation_groups = {'en': [], 'de': []}
translation_fragments = {'en': [], 'de': []}

with open('meter_value_group.csv', newline='', encoding='utf-8') as f:
    for row in csv.DictReader(f):
        if len(row['measurand']) == 0:
            # skip empty
            continue

        name = ' '.join([part for part in list(row.values())[:4] if len(part) > 0])
        identifier = name.replace(' ', '_').lower()
        display_name_en       = escape(row['display_name_en'].replace('\"', '\\"') if len(row['display_name_en']) > 0 else ("TRANSLATION_MISSING " + name))
        display_name_en_muted = escape(row['display_name_en_muted'].replace('\"', '\\"'))
        display_name_de       = escape(row['display_name_de'].replace('\"', '\\"') if len(row['display_name_en']) > 0 else ("TRANSLATION_MISSING " + name))
        display_name_de_muted = escape(row['display_name_de_muted'].replace('\"', '\\"'))

        translation_groups['en'].append(f'"group_{identifier}": {display_name_en}')
        translation_groups['en'].append(f'"group_{identifier}_muted": {display_name_en_muted}')
        translation_groups['de'].append(f'"group_{identifier}": {display_name_de}')
        translation_groups['de'].append(f'"group_{identifier}_muted": {display_name_de_muted}')

with open('meter_value_fragment.csv', newline='', encoding='utf-8') as f:
    for row in csv.DictReader(f):
        if len(row['fragment']) == 0:
            # skip empty
            continue

        identifier = row['fragment'].replace(' ', '_').lower()
        display_name_en = escape(row['display_name_en'].replace('\"', '\\"') if len(row['display_name_en']) > 0 else ("TRANSLATION_MISSING " + row['fragment']))
        display_name_de = escape(row['display_name_de'].replace('\"', '\\"') if len(row['display_name_en']) > 0 else ("TRANSLATION_MISSING " + row['fragment']))

        translation_fragments['en'].append(f'"fragment_{identifier}": {display_name_en}')
        translation_fragments['de'].append(f'"fragment_{identifier}": {display_name_de}')

def update_value_id_tree(sub_tree, sub_id):
    key = sub_id[0]

    if len(sub_id) > 2:
        update_value_id_tree(sub_tree.setdefault(key, OrderedDict()), sub_id[1:])
    else:
        sub_tree[key] = sub_id[1]

tree_paths = []

with open('meter_value_id.csv', newline='', encoding='utf-8') as f:
    for row in csv.DictReader(f):
        if len(row['id']) == 0:
            # skip empty
            continue

        id_ = row['id']
        name_list = [row[x] for x in ['measurand', 'submeasurand', 'phase', 'direction', 'kind']]
        name = ' '.join([x for x in name_list if len(x) > 0 and not x.startswith('*')])
        name_without_phase = ' '.join([row[x] for x in ['measurand', 'submeasurand', 'direction', 'kind'] if len(row[x]) > 0 and not row[x].startswith('*')])
        phase = row['phase']
        identifier = name.replace(' ', '')
        identifier_without_phase = name_without_phase.replace(' ', '_').lower()
        unit = row['unit']
        digits = row['digits']
        display_name_en       = escape(row['display_name_en'].replace('\"', '\\"') if len(row['display_name_en']) > 0 else ("TRANSLATION_MISSING " + name))
        display_name_en_muted = escape(row['display_name_en_muted'].replace('\"', '\\"'))
        display_name_de       = escape(row['display_name_de'].replace('\"', '\\"') if len(row['display_name_de']) > 0 else ("TRANSLATION_MISSING " + name))
        display_name_de_muted = escape(row['display_name_de_muted'].replace('\"', '\\"'))

        tree_path = [part.replace(' ', '_').replace('*', '').lower() for part in name_list if len(part) > 0] + [identifier]
        tree_path_flat = '.'.join(tree_path[:-1])

        update_value_id_tree(value_id_tree, tree_path)

        if tree_path_flat in tree_paths:
            raise Exception(f'tree path {tree_path_flat} is not unique')

        tree_paths.append(tree_path_flat)

        value_id_enum.append(f'    {identifier} = {id_}, // {unit}\n')
        value_id_list.append(f'    MeterValueID.{identifier},\n')
        value_id_infos.append(f'    /* {identifier} */ {id_}: {{unit: "{unit}", digits: {digits}, tree_path: {tree_path[:-1]}}},\n')
        translation_values['en'].append(f'"value_{id_}": {display_name_en}')
        translation_values['en'].append(f'"value_{id_}_muted": {display_name_en_muted}')
        translation_values['de'].append(f'"value_{id_}": {display_name_de}')
        translation_values['de'].append(f'"value_{id_}_muted": {display_name_de_muted}')

        for phases in groupings:
            if phase in phases:
                for foobar in value_id_order:
                    if foobar[1] == identifier_without_phase and foobar[2] == phases:
                        foobar[0].append(identifier)
                        break
                else:
                    value_id_order.append([[identifier], identifier_without_phase, phases])

                break
        else:
            value_id_order.append([[identifier], None, None])

import pprint
pprint.pprint(tree_paths)
for i, tree_path in enumerate(tree_paths):
    for k, other in enumerate(tree_paths):
        if i == k:
            continue

        if other.startswith(tree_path):
            raise Exception(f'tree path {tree_path} is prefix of {other}')

value_id_order_str = []

for foobar in value_id_order:
    foobar_str = f'    {{ids: [{", ".join([f"MeterValueID.{id_}" for id_ in foobar[0]])}], group: '

    if foobar[1] != None:
        foobar_str += f'"{foobar[1]}"'
    else:
        foobar_str += 'null'

    foobar_str += ', phases: '

    if foobar[2] != None:
        foobar_str += f'"{", ".join([x.replace(" ", "-") for x in foobar[2]])}"'
    else:
        foobar_str += 'null'

    foobar_str += '},\n'

    value_id_order_str.append(foobar_str)

with open('meter_value_id.h', 'w') as f:
    f.write('// WARNING: This file is generated.\n\n')
    f.write('#pragma once\n\n')
    f.write('enum class MeterValueID {\n')
    f.write(''.join(value_id_enum))
    f.write('};\n')

def format_value_id_tree(sub_tree, indent):
    result = ''

    for sub_id in sub_tree.items():
        result += f"{'    ' * (indent + 1)}'{sub_id[0]}': "

        if isinstance(sub_id[1], dict):
            result += f"{{\n{format_value_id_tree(sub_id[1], indent + 1)}{'    ' * (indent + 1)}}},\n"
        else:
            result += f"MeterValueID.{sub_id[1]},\n"

    return result

with open('../../../web/src/modules/meters/meter_value_id.ts', 'w', encoding='utf-8') as f:
    f.write('// WARNING: This file is generated.\n\n')
    f.write('export const enum MeterValueID {\n')
    f.write(''.join(value_id_enum))
    f.write('}\n\n')
    f.write('export const METER_VALUE_IDS: MeterValueID[] = [\n')
    f.write(''.join(value_id_list))
    f.write(']\n\n')
    f.write('export const METER_VALUE_INFOS: {[id: number]: {unit: string, digits: 0|1|2|3, tree_path: string[]}} = {\n')
    f.write(''.join(value_id_infos))
    f.write('}\n\n')
    f.write('export const METER_VALUE_ORDER: {ids: MeterValueID[], group: string, phases: string}[] = [\n')
    f.write(''.join(value_id_order_str))
    f.write(']\n\n')
    f.write('export type MeterValueTreeType = {[key: string]: MeterValueTreeType | MeterValueID};\n\n')
    f.write('export const METER_VALUE_TREE: MeterValueTreeType = {\n')
    f.write(format_value_id_tree(value_id_tree, 0))
    f.write('}\n')

for lang in translation_values:
    util.specialize_template(f'../../../web/src/modules/meters/translation_{lang}.tsx.template', f'../../../web/src/modules/meters/translation_{lang}.tsx', {
        '{{{values}}}': ',\n            '.join(translation_values[lang]),
        '{{{groups}}}': ',\n            '.join(translation_groups[lang]),
        '{{{fragments}}}': ',\n            '.join(translation_fragments[lang]),
    })

# NEVER EVER EDIT OR REMOVE IDS. Only append new ones. Changing or removing IDs is a breaking API and config change!
classes = [
    'None',
    'RS485 Bricklet',
    'EVSE V2',
    'Energy Manager',
    'API',
    'Sun Spec',
    'Modbus TCP',
    'MQTT Subscription',
]

class_values = []

for i, name in enumerate(classes):
    class_values.append('    {0} = {1},\n'.format(util.FlavoredName(name).get().camel, i))

with open('meters_defs.h', 'w') as f:
    f.write('// WARNING: This file is generated.\n\n')
    f.write('#include <stdint.h>\n\n')
    f.write('#pragma once\n\n')
    f.write('enum class MeterClassID : uint8_t {\n')
    f.write(''.join(class_values))
    f.write('};\n\n')
    f.write(f'#define METER_CLASSES {len(class_values)}')

with open('../../../web/src/modules/meters/meters_defs.ts', 'w') as f:
    f.write('// WARNING: This file is generated.\n\n')
    f.write('export const enum MeterClassID {\n')
    f.write(''.join(class_values))
    f.write('}\n')
