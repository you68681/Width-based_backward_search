# Algorithms in Width-Based Backward Search [ICAPS 2021 paper]

### Getting Started ###

Please first install LAPKT by following this instructions: https://github.com/you68681/Width-based_backward_search/blob/main/LAPKT-public/README.md

Please also define LAPKT_PATH as an enviromnent variable. Add the following line to your .bash or .profile file or simply execute it in your terminal:
```
  export LAPKT_PATH = /Absolute-path-to-LAPKT-folder
```

Please use FF parser when running the code.


### Options ###

```
  --help                                Show help message. 
  --domain arg                          Input PDDL domain description
  --problem arg                         Input PDDL problem description
  --output arg                          Output file for plan
  --max_novelty arg (=2)                Max width w for novelty (default 2)
```

# Running forward BFWS (Best First Width Search) #

```
Search Algorithms::

  --BFWS-f5 arg (=0)                        BFWS(f5) where f5= (w,#g), w_{#r,#g}, as in AAAI-17 paper

Polynomial Search Algorithms::
 
  --k-BFWS arg (=0)                         k-BFWS(f5), pruning w > k, where k = bound() argument, default 2
```

# Running backward BFWS #

```
Search Algorithms::

  --BFWS-f5-backward arg (=0)               Backward BFWS(f5) as in ICAPS-21 paper

Polynomial Search Algorithms::
 
  --k-BFWS-backward arg (=0)                Backward k-BFWS(f5), pruning w > k, where k = bound() argument, default 2
```

# Running width-based bidirectional search algorithms #

Backward k-BFWS(f5) and Forward k-BFWS(f5) alternate 1-step in each direction

## Running k-BDWS-e (front-to-end version) #

front-to-end guides the search with opposite goal state.

```
Polynomial Search Algorithms::
 
  --k-BDWS-e arg (=0)                      Check meets-in-the-middle with respect to the novelty 1 frontier of the opposite direction, default 2
  
  --k-BDWS-e-head arg (=0)                 Check meets-in-the-middle with respect to the last expanded state in the opposite direction, default 2
  
  --k-BDWS-e-close arg (=0)                Check meets-in-the-middle with respect to the full close list, default 2
  
  --k-BDWS-e-random arg (=0)               Check meets-in-the-middle with respect to a random state in the close list, default 2
```

## Running k-BDWS-f (front-to-front version) ##

front-to-fnd guides the search with last expanded state by opposite direction.

```
Polynomial Search Algorithms::
 
  --k-BDWS-f arg (=0)                      Check meets-in-the-middle with respect to the novelty 1 frontier of the opposite direction, default 2
  
  --k-BDWS-f-head arg (=0)                 Check meets-in-the-middle with respect to the last expanded state in the opposite direction, default 2
  
  --k-BDWS-f-close arg (=0)                Check meets-in-the-middle with respect to the full close list, default 2
```

## Running FB ##

```
Polynomial Search Algorithms::

--forward-backward (=0)                    Forward k-BFWS(f5) first and then backward backward k-BFWS(f5) if forward search stops with no solution, default 2

Search Algorithms::

--DUAL-FB arg (=1)                         Forward-backward first and then BFWS using h_ff and h_landcount as in AAAI-17 paper
```


### Credits ###

This project is a joint work by Chao Lei and Nir Lipovetzky

### Paper ###
You can read more about it in the [ICAPS 2021 paper](https://ojs.aaai.org/index.php/ICAPS/article/view/15965/15776) and [AAAI 2017 paper](http://people.eng.unimelb.edu.au/nlipovetzky/papers/aaai17-BFWS-novelty-exploration.pdf) and [ICAPS 2017 paper](http://people.eng.unimelb.edu.au/nlipovetzky/papers/icaps17-polytime-BFWS.pdf)



  
