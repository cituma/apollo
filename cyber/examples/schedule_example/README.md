
run command on a terminal:
$ mainboard -p schedule_example_chore  -d ${DAG_PATH}/component_d/d.dag -d ${DAG_PATH}/component_b/b.dag -d ${DAG_PATH}/component_c/c.dag -d ${DAG_PATH}/component_a/a.dag

run this command on another terminal:
$ mainboard -d ${DAG_PATH}/component_input/input.dag
