# GADEN Simulator Configuration

The launch file passes the `base_path` variable to the simulator.

## The root level of the config file

The following keywords are supported on the root level of the config file:

### `occupancy_grid_file`

The simulator looks in `base_path + occupancy_grid_file` for the occupancy grid. The file extension of `occupancy_grid_file` should be `.vdb`.

If the simulator cannot find the specified file, the preprocessor is started to generate it.

## The `environment` section in the config file

The following keywords are supported in the `environment` section of the config file:

### `format`

Specifies in which format the environment is provided. Currently the following formats are supported:

- ``


## The `visualisation` section in the config file

The following keywords are supported in the `visualisation` section of the config file:


