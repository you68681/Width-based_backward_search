LAPKT
======

Lightweight Automated Planning Toolkit is a joint work by Miquel Ramirez <miquel.ramirez@gmail.com>, Nir Lipovetzky <nirlipo@gmail.com> and Christian Muise <christian.muise@gmail.com> with Last update: September 2014

CONTENTS
========

1. Overview of toolkit components
2. Building LAPKT
3. Bakcward Contributing

1 - OVERVIEW
===========

LAPKT separates search engines from the data structures used to represent
planning tasks. This second component receives the name of 'interface' since
it is indeed the interface that provides the search model to be solved.

2 - BUILDING LAPKT
==================

Please refer to https://github.com/LAPKT-dev/LAPKT-public to build and learn how to use LAPKT.



3 - Bakcward Contributing
===========


In ICAPS-21 paper the following interfaces are modified:

* 'agnostic': a interface easy to wrap PDDL parsers

* 'ff': this interface wraps FF parsing components to obtain 'agnostic' looking
tasks.


### Paper ###
You can read more about it in the [ICAPS 2021 paper](https://ojs.aaai.org/index.php/ICAPS/article/view/15965/15776)
