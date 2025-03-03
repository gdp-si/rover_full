"""Parameter substitution for multiple robots."""
import os
import tempfile
from typing import Dict, Optional

import launch
import yaml
from nav2_common.launch import RewrittenYaml


def update_navigation_params(
    params_file: str,
    param_substitutions: dict,
    namespace: str,
    param_callback: callable = None,
):
    """Update navigation parameters for multiple robots."""
    assert os.path.isfile(
        params_file
    ), f"Navigation params file not found: {params_file}"

    parameters = yaml.load(
        open(file=params_file, mode="r", encoding="utf-8"), Loader=yaml.FullLoader
    )
    if param_callback is not None:
        param_substitutions.update(param_callback(namespace))

    rewritten_yaml = tempfile.NamedTemporaryFile(mode="w", delete=False)
    configs = ParameterSubstitutor(
        source_file=params_file,
        param_rewrites=param_substitutions,
        root_key=namespace,
        convert_types=True,
    )
    configs.substitute_params(parameters, param_substitutions)
    configs.substitute_keys(parameters, {})
    if namespace != "" and namespace != "/":
        parameters = {namespace: parameters}

    yaml.dump(parameters, rewritten_yaml)
    rewritten_yaml.close()
    print(rewritten_yaml.name)  # TODO: remove this line

    return rewritten_yaml.name


class DictItemReference:
    """Reference to a dictionary item."""

    def __init__(self, dictionary, key):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self):
        """Return the key of the dictionary item."""
        return self.dictKey

    def set_value(self, value):
        """Set the value of the dictionary item."""
        self.dictionary[self.dictKey] = value


class ParameterSubstitutor(RewrittenYaml):
    """Class to substitute parameters in the yaml file."""

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: Dict,
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[Dict] = None,
        convert_types=False,
    ) -> None:
        super().__init__(
            source_file, param_rewrites, root_key, key_rewrites, convert_types
        )
        self.__convert_types = convert_types

    def substitute_params(self, yaml, param_rewrites):
        """Substitute parameters in the yaml file."""
        # substitute leaf-only parameters
        for key in self.get_yaml_leaf_kays(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.set_value(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split(".")
                yaml = self.update_yaml_path_values(yaml, yaml_keys, rewrite_val)

    def get_yaml_leaf_kays(self, yamlData):
        """Get the leaf keys of a yaml file."""
        try:
            for key in yamlData.keys():
                for k in self.get_yaml_leaf_kays(yamlData[key]):
                    yield k
                yield DictItemReference(yamlData, key)
        except AttributeError:
            return

    def pathify(self, d, p=None, paths=None, joinchar="."):
        """Pathify a dictionary."""
        if p is None:
            paths = {}
            self.pathify(d, "", paths, joinchar=joinchar)
            return paths
        pn = p
        if p != "":
            pn += "."
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, pn + k, paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d

    def convert(self, text_value):
        """Convert a string to a boolean, int, float, or relevant types."""
        if isinstance(text_value, list):
            return [self.convert(x) for x in text_value]

        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if "." in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == "true":
            return True
        if text_value.lower() == "false":
            return False

        # nothing else worked so fall through and return text
        return text_value

    def update_yaml_path_values(self, yaml, yaml_key_list, rewrite_val):
        """Update the yaml value at the given path with the given value."""
        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            yaml[key] = self.update_yaml_path_values(
                yaml.get(key, {}), yaml_key_list, rewrite_val
            )

        return yaml
