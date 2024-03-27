# Problem 1

- `domain1.pddl`
- `problem1.pddl`
- planner output: `sas_plan_lamafirst`, `sas_plan_astar_lmcut`

--- 

Planner used: Fast Downward in planutils.

With lama-first pre-defined configuration:
```console
downward --alias lama-first domain1.pddl problem1.pddl
```

With A* with landmarks heuristic:
```console
downward domain1.pddl problem1.pddl --search "astar(lmcut())"
```