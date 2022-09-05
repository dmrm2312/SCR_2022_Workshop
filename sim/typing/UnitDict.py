import copy
import logging
import time

import numpy as np
from typing import Any
from sim.typing.UnitType import UnitType
from typing import Union

SEPARATOR = '.'


class Default(UnitType):
    # unit
    default_unit = 'dimensionless'
    # default value for this unit, if none then we do dtype(), i.e. float() = 0.0
    default_value = None
    # data type
    default_dtype = float
    # dimension i.e. 1D array of 3 columns -> 3 or 2D array 2 by 3 -> (2, 3)
    default_dim: Union[int, tuple[int, int]] = 1
    # data range for this unit
    default_drange: tuple[(Union[int, float, str], Union[int, float, str])] = (float('-inf'), float('inf'))
    # data mapping to let you create your own unit conversion
    # unit='count'; drange=(0, 4095); drange_map=('0.01m', '1m')
    # This indicates 0 count corresponds to 0.01m and 4096 to 1m and inbetween is linear interpolation
    # drange_map=('-1m', func, '1m') and takes func(val, sv, lv). this let you define custom interpolation
    # you can also have more points. If no func is specified, then it'll be linear interpolation
    # drange=(0, 1024, 2048, 3072 ,4096); drange_map=('10mm', '0.1m', '0.5m', func, '0.75m', ,'1m')
    default_drange_map: tuple[(Union[int, float, str], Union[int, float, str])] = None
    # data scale. (-1, 1) -> -100% to 100%. (0, 1) -> 0% to 100%
    defualt_drange_scale: tuple[(Union[int, float], Union[int, float])] = (-1, 1)


class DefDict:
    reserved = ['definition', 'get', 'set', 'keys', 'list', 'list_keys', 'ndtall']

    def __init__(self, definition, dtype=Any, name=None, prefixes=None, suffixes=None, format_rule=None, shape=None,
                 rules=None, nested_def=True):
        self._definition = dict()
        self._data = dict()
        self._name = name
        self.format_rule = format_rule
        self.shape = shape
        self.suffixes = []
        self.prefixes = []
        self.dtype = dtype
        if not isinstance(rules, list) and rules is not None:
            rules = [rules]
        self.rules = rules
        self.nested_def = nested_def

        if isinstance(suffixes, dict):
            suffixes = list(suffixes.keys())
        elif not isinstance(suffixes, list):
            suffixes = [suffixes]

        if not isinstance(definition, tuple):
            definition = (definition,)
        for d in definition:  # add as many definition as you want
            try:
                d = self._unpack_dict(**d)  # in case user forgot **definitino
            except TypeError:
                pass
            suffixes.extend(self.add_definition(d, self.dtype, self.nested_def))
            if isinstance(d, DefDict) and suffixes:
                self._add_suffix(d.list_keys())

        if prefixes is not None:
            if isinstance(prefixes, dict):
                prefixes = list(prefixes.keys())
            self._add_prefixes(prefixes)
        if suffixes is not None:
            self._add_suffixes(suffixes)

    def _unpack_dict(self, definition, dtype=Any, name=None, prefixes=None, suffixes=None, format_rule=None, shape=None,
                     rules=None, nested_def=True):
        self.dtype = dtype
        self._name = name
        self.prefixes.append(prefixes)
        self.suffixes.append(suffixes)
        self.format_rule = format_rule
        self.shape = shape
        self.add_rule(rules)
        self.nested_def = nested_def
        return definition

    def ndarray(self, reshape: tuple = None):
        data_list = self.list()
        if reshape is None:
            reshape = self.shape
        return np.array(data_list).reshape(reshape)

    def ndtall(self):
        return self.ndarray((len(self._data), 1))

    def list(self):
        return list(self.values())

    def list_keys(self):
        return list(self.keys())

    def get(self, key=None):
        if key is None:
            ret = self.list()[0]  # get the first element when no key has been specified
        else:
            ret = self._data[key][0]
        return ret

    def dict(self):
        return self.data  # return dictionary

    def keys(self, to_int=False):
        if to_int:
            return map(int, self.list_keys())
        else:
            return self.data.keys()

    def as_ruled(self):
        if self.format_rule is None:
            return self.data
        return self.format_rule.bind(self.data)

    def remove_prefix(self, prefix=None):
        d = copy.deepcopy(self)
        d.clear()
        for k in self.list_keys():
            k_seq = k.split(SEPARATOR)
            if len(k_seq) <= 1:
                break
            if prefix is None:
                k_seq.pop(0)
                d.add_definition({SEPARATOR.join(k_seq): self.DEF[k]})
                d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
            else:
                if prefix in k_seq:
                    k_seq.remove(prefix)
                    d.add_definition({SEPARATOR.join(k_seq): self.DEF[k]})
                    d._data[SEPARATOR.join(k_seq)] = self._data.get(k)
        return d

    def _add_prefixes(self, prefixes):
        if not isinstance(prefixes, list):
            prefixes = [prefixes]
        for name in prefixes:
            self._add_prefix(name)

    def _add_prefix(self, name):
        if name in self.reserved:
            raise AttributeError(f'the prefix {name} is reserved for DefDict')
        if name in self.prefixes:
            logging.debug(f'the prefix {name} is already registered')
            return
        self.prefixes.append(name)

        def method(self, ids=None):
            if ids is None:
                return self.remove_prefix(name)
            elif not isinstance(ids, list):
                ids = [ids]
            return self.remove_prefix(name).filter(ids)

        setattr(self, name, method.__get__(self))

    def _add_suffixes(self, suffixes):
        if not isinstance(suffixes, list):
            suffixes = [suffixes]
        for name in suffixes:
            if name is None:
                continue
            self._add_suffix(name)

    def _add_suffix(self, name):
        if name in self.suffixes:
            logging.debug('the suffix is already registered')
            return
        self.suffixes.append(name)

        def method(self):
            d = copy.deepcopy(self)
            d.clear()
            for k, v in self.items():
                if isinstance(v, DefDict):
                    d.add_definition({k: v.DEF[name]})
                    d._data[k] = v._data.get(name)
                elif isinstance(v, dict):
                    d.add_definition({k: v[name]})
                    d._data[k] = v.get(name)
                elif k == name:
                    d.add_definition({k: self.DEF[k]})
                    d._data[k] = v
            return d

        setattr(self, name, method.__get__(self))

    @property
    def data(self):
        ret = {}
        for k, v in self._data.items():
            if not k.startswith('_'):  # protected keys are not accessible normally
                ret[k] = v[0]
        return ret

    @data.setter
    def data(self, ndata):
        self.set(ndata)

    def all(self):  # return in dict including protected values
        return {k: v[0] for k, v in self._data.items()}

    def set(self, ndata):
        if ndata is None:
            return self
        if self.rules is not None:
            ret = self._apply_rules(ndata)
            if ret is not None:
                ndata = ret
        if self.format_rule is not None:
            try:
                ndata = self.format_rule.inv_bind(ndata)
            except:
                pass
        if isinstance(ndata, DefDict):
            self._from_defdict(ndata)
        if isinstance(ndata, dict):
            self._dict2dict(ndata)
        elif isinstance(ndata, list):
            self._list2dict(ndata)
        elif isinstance(ndata, tuple):
            ndata = list(ndata)
            self._list2dict(ndata)
        elif isinstance(ndata, np.ndarray):
            self._list2dict(ndata)
        else:
            self._list2dict([ndata])
        return self

    def _apply_rules(self, data):
        ret = None
        if isinstance(data, dict) or isinstance(data, DefDict):
            for rule in self.rules:
                if rule.origin is None:
                    continue
                if rule.if_rule_apply(data.keys()):
                    ret = rule.map(data)
                    if ret is not None:
                        break
        return ret

    def add_rule(self, rule):
        if self.rules is None:
            self.rules = []
        self.rules.append(rule)
        return self

    def clear_rules(self):
        self.rules = None
        return self

    def set_name(self, name):
        self._name = name
        return self

    def format(self, ndata):
        """format does not change the stored data, but format the input into current definition"""
        st = time.perf_counter()
        d = copy.deepcopy(self)
        print(time.perf_counter() - st)
        return d.set(ndata)

    @property
    def DEF(self):
        return self._definition

    def get_DEF(self, keys):
        key_lists = []
        if isinstance(keys, DefDict):
            key_lists = keys.list_keys()
        elif isinstance(keys, dict):
            key_lists = list(keys.keys())
        elif isinstance(keys, tuple):
            k, v = keys
            key_lists = [k]

        DEFs = []
        for k in key_lists:
            if k in self.DEF.keys():
                DEFs.append(self.DEF[k])
            else:
                raise (f'{k} is not in definition')
        return DEFs

    def add_name(self, name):
        self._name = name
        return self

    def add_definition(self, ndef, dtype=Any, nested_dict=False):
        keys = []
        suffixes = []
        if isinstance(ndef, dict):
            for k, v in ndef.items():
                if isinstance(v, dict) and nested_dict:
                    v = DefDict(v)
                    suffixes.extend(v.list_keys())
                if isinstance(v, type) or v is Any:
                    self._definition[k] = self._to_UnitType(v)
                    keys.append(k)
                else:
                    self._definition[k] = self._to_UnitType(v)
                    self._data[k] = [v]  # initiate with given values

        elif isinstance(ndef, list):
            self._definition.update(dict.fromkeys(ndef, self._to_UnitType(dtype)))
            keys.extend(ndef)
        elif isinstance(ndef, str):
            self._definition[ndef] = self._to_UnitType(dtype)
            keys.append(ndef)
        else:
            raise TypeError('You can only add str, dict, or list')
        self.init_data(keys)
        return suffixes

    def _to_UnitType(self, dtype):
        if not isinstance(dtype, UnitType):
            if isinstance(dtype, type) and issubclass(dtype, UnitType):
                dtype = dtype()
            else:  # if not UnitType then default (dimensionless) is used
                if isinstance(dtype, DefDict):
                    dtype = dtype  # if some instance is handover, try to get type i.e. type(0.0) -> float
                    dtype = Default(dtype=dtype, default=dtype.dict())
                elif not isinstance(dtype, type):
                    val = dtype
                    dtype = type(dtype)  # if some instance is handover, try to get type i.e. type(0.0) -> float
                    dtype = Default(dtype=dtype, default=val)
                else:
                    dtype = Default(dtype=dtype)
        return dtype

    def init_data(self, keys):
        for k, v in self._definition.items():
            if k in keys:
                self._data[k] = [v.default]  # use UnitType Default

    def _from_defdict(self, data):
        for k, v in data.items():
            if k in self._data.keys():
                if isinstance(self.DEF[k], DefDict):
                    self._data[k][0].set(v)
                else:
                    self._data[k][0] = self._enforce_type(self.DEF[k], v, data.DEF[k])

    def _dict2dict(self, data: dict):
        stored_data_keys = []
        for k, v in data.items():
            if k in self._data.keys():
                DEF = self.DEF[k]
                if isinstance(DEF, DefDict):
                    self._data[k][0].set(v)
                else:
                    if isinstance(v, UnitType):
                        self._data[k][0] = self._enforce_type(self.DEF[k], v.data, v)
                    else:
                        self._data[k][0] = self._enforce_type(self.DEF[k], v)
                stored_data_keys.append(k)

    def _list2dict(self, data):
        length = min(len(data), len(self._definition)) - 1
        for i, key in enumerate(self._data.keys()):
            if i > length:
                break
            if key.startswith('_'):
                continue
            DEF = self.DEF[key]
            if isinstance(DEF, DefDict):
                if self._check_if_iterable(data[i]):  # if data is iterative, then just hand over to sub DefDict
                    sub_data = data[i]
                else:  # otherwise, try to collect data
                    # DefDict(x, quat=DefDict(w,x,y,z)).set([0, 1, 0, 0])
                    # then user probably meant x = 0, quat = [1, 0, 0, 0]
                    try:
                        sub_data = data[i:i + DEF.dim]
                    except IndexError:
                        # if there is not enough elements, still hand them over(May sub DefDict reject it tho)
                        sub_data = data[i:-1]
                if DEF.strict and len(sub_data) != DEF.dim:  # if strict, it should receive the dim length
                    continue
                self._data[key][0].set(sub_data)  # call sub DefDict.set() and it'll do the rest of the job
            else:
                self._data[key][0] = self._enforce_type(self.DEF[key], data[i])

    def _check_if_iterable(self, item):
        try:
            iter(item)
            return True
        except TypeError:
            return False

    def _enforce_type(self, dtype: UnitType, value, vdef=None):
        return dtype.to(value, vdef)  # UnitType will handle unit conversion and type enforcement

    def bind(self, bind_rule, val=None):
        if val is None:
            val = self.dict()
        self.set(bind_rule.bind(val))
        return self

    def map(self, map_rule, val=None):
        return self.bind(map_rule, val=None)

    def inv_bind(self, bind_rule, val=None):
        if val is None:
            val = self.dict()
        self.set(bind_rule.inv_bind(val))
        return self

    def assert_data(self, data=None):
        if data is None:
            data = self._data
        else:
            data = dict(data)
        assert (len(data) == len(self.DEF))
        assert (all(type(x) == y for x, y in zip(data.values(), self.DEF.values())))

    def _to_key_list(self, keys):
        if isinstance(keys, dict):
            keys = list(keys.keys())
        if isinstance(keys, str):
            keys = [keys]
        if isinstance(keys, int):
            keys = [str(keys)]
        if not isinstance(keys, list):
            raise TypeError('keys must be either string, int, list, or dict')
        return keys

    def filter(self, keys):  # ToDo support all iteratibe
        key_list = []
        if isinstance(keys, tuple):
            for t in keys:
                key_list.extend(self._to_key_list(t))
        else:
            key_list.extend(self._to_key_list(keys))

        key_list = map(str, key_list)
        d = copy.deepcopy(self)
        d.clear()
        for k in key_list:
            if k in self.keys():
                d.add_definition({k: self.DEF[k]})
                d._data[k] = self._data.get(k)
        return d  # DefDict

    def filter_data(self, data):
        if isinstance(data, dict):
            data = list(data.values())
        if not isinstance(data, list):
            data = [data]

        found = []
        for i, value in enumerate(self.list()):
            for search_for in data:
                if value == search_for:
                    found.append(i)
        keys = list(self.keys())
        vals = list(self.values())
        d = copy.deepcopy(self)
        d.clear()
        for index in found:
            d.add_definition({keys[index]: vals[index]})
            d._data[keys[index]] = vals[index]
        return d

    def flat_list(self):
        ret = []
        for elem in self.values():
            try:
                i = iter(elem)
                for e in elem:
                    ret.append(e)
            except TypeError:
                ret.append(elem)
        return ret

    @staticmethod
    def is_DefDict():
        return True

    ###################################################################
    # dictionary methods
    def clear(self):  # real signature unknown; restored from __doc__
        """ D.clear() -> None.  Remove all items from D. """
        self._data.clear()
        self.DEF.clear()

    def copy(self):  # real signature unknown; restored from __doc__
        """ D.copy() -> a shallow copy of D """
        return copy.copy(self)

    def clone(self):
        """
        Deep copy of itself
        """
        return copy.deepcopy(self)

    def items(self):  # real signature unknown; restored from __doc__
        """ D.items() -> a set-like object providing a view on D's items """
        return self.data.items()

    def pop(self, keys, d=None):  # real signature unknown; restored from __doc__
        """
        D.pop(k[,d]) -> v, remove specified key and return the corresponding value.

        If key is not found, default is returned if given, otherwise KeyError is raised
        """
        rets = []
        if not isinstance(keys, list):
            k = [keys]
        for k in keys:
            if k in self._data.keys():
                rets.append(self._data.get(k)[0])
                self.init_data(k)
            else:
                rets.append(d)
        return rets

    def popitem(self, *args, **kwargs):  # real signature unknown
        """
        Remove and return a (key, value) pair as a 2-tuple.

        Pairs are returned in LIFO (last-in, first-out) order.
        Raises KeyError if the dict is empty.
        """
        keys = [k for k, v in args]
        self.pop(keys)

    def setdefault(self, ndata):  # real signature unknown
        """
        Insert key with a value of default if key is not in the dictionary.
        Return the value for key if key is in the dictionary, else default.
        """
        return self.set(ndata)

    def update(self, ndata):  # known special case of dict.update
        """
        same as set
        """
        return self.set(ndata)

    def values(self):  # real signature unknown; restored from __doc__
        """ D.values() -> an object providing a view on D's values """
        return self.data.values()

    def __setitem__(self, key, value):
        if key in self.DEF.keys():
            if isinstance(self._data[key][0], DefDict):  # for nested def dict
                self._data[key][0].set(value)
            else:
                self._data[key][0] = value
        else:
            raise KeyError(f'Key {key} is not in definition')

    def __getitem__(self, item):
        return self._data.__getitem__(item)[0]

    ##############################################################
    # magic methods
    def __len__(self):
        return len(self._data)

    def __repr__(self):
        return {k: v[0].__str__() for k, v in self._data.items()}.__str__()

    def __str__(self):
        return {k: v[0].__str__() for k, v in self._data.items()}.__str__()

    ##############################################################
    # Math operations
    def _math(self, other, func, immutable=True):
        if immutable:
            current = self.clone()
        else:
            current = self
        if np.isscalar(other):  # if scalar, then add the value to all elements
            for k in current._data.keys():
                current._data[k][0] = func(current._data[k][0], other)
        else:  # other wise element wise
            if not isinstance(other, DefDict):
                # if not DefDict, create one assuming
                other_defdict = self.clone()
                other_defdict.init_data(other_defdict.list_keys())
                other_defdict.set(other)
            else:
                other_defdict = other
            # sum for corresponding keys
            for k, o in zip(current.filter(other_defdict.keys()).list_keys(),
                            other_defdict.filter(other_defdict.list_keys()).list()):
                current._data[k][0] = func(current._data[k][0], o)
        return current

    def __iter__(self):
        return iter(self.values())

    def __add__(self, other):
        return self._math(other, lambda v, o: v + o, immutable=True)

    def __iadd__(self, other):
        return self._math(other, lambda v, o: v + o, immutable=False)

    def __sub__(self, other):
        return self._math(other, lambda v, o: v - o, immutable=True)

    def __isub__(self, other):
        return self._math(other, lambda v, o: v - o, immutable=False)

    def __mul__(self, other):
        return self._math(other, lambda v, o: v * o, immutable=True)

    def __imul__(self, other):
        return self._math(other, lambda v, o: v * o, immutable=False)

    def __floordiv__(self, other):
        return self._math(other, lambda v, o: v // o, immutable=True)

    def __ifloordiv__(self, other):
        return self._math(other, lambda v, o: v // o, immutable=False)

    def __truediv__(self, other):
        return self._math(other, lambda v, o: v / o, immutable=True)

    def __itruediv__(self, other):
        return self._math(other, lambda v, o: v / o, immutable=False)

    def __mod__(self, other):
        return self._math(other, lambda v, o: v % o, immutable=True)

    def __imod__(self, other):
        return self._math(other, lambda v, o: v % o, immutable=True)

    def __pow__(self, other, modulo=None):  # Todo: modulo implementation
        return self._math(other, lambda v, o: v ** o, immutable=True)

    def __ipow__(self, other, modulo=None):  # Todo: modulo implementation
        return self._math(other, lambda v, o: v ** o, immutable=False)

    #############################
    def _compare(self, other, func):
        ret = []
        current = self
        if np.isscalar(other):  # if scalar, then compare the value to all elements
            for k in current._data.keys():
                ret.append(func(current._data[k][0], other))
        else:  # other wise element wise
            if not isinstance(other, DefDict):
                # if not DefDict, create one assuming
                other_defdict = self.clone()
                other_defdict.init_data(other_defdict.list_keys())
                other_defdict.set(other)
            else:
                other_defdict = other
            # sum for corresponding keys
            for k, o in zip(current.filter(other_defdict.keys()).list_keys(),
                            other_defdict.filter(other_defdict.list_keys()).list()):
                ret.append(func(current._data[k][0], o))
        return ret

    def __lt__(self, other):
        return self._compare(other, lambda v, o: v < o)

    def __le__(self, other):
        return self._compare(other, lambda v, o: v <= o)

    def __eq__(self, other):
        return self._compare(other, lambda v, o: v == o)

    def __ne__(self, other):
        return self._compare(other, lambda v, o: v != o)

    def __ge__(self, other):
        return self._compare(other, lambda v, o: v >= o)

    def __gt__(self, other):
        return self._compare(other, lambda v, o: v > o)

    def __round__(self, n=None):
        d = self.clone()
        for k, v in d.items():
            d._data[k][0] = round(v, n)
        return d


if __name__ == '__main__':
    d = DefDict({'leg.0': DefDict({'j.0': 1, 'j.1': 2}, prefixes=['j']),
                 'leg.1': DefDict({'j.0': 1, 'j.1': 2}, prefixes=['j'])}, prefixes=['leg'], suffixes=['j'])
    print(d)