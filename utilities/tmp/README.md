# Note on the `utilities/tmp` directory

- content of `utilities/tmp` are meant to be modified/deleted at runtime
- `tmp/dockerized-norlab-project-mock` is a placeholder directory (might be excluded in IDE). 
  It is needed to implement `project-deploy` cloning/copying/checkout functionalities in and out of 
  docker container.  
  For that reason, it is cloned as a full repository at runtime, not a submodule.  
  - In DNP, cloning and removing is manage by `setup_mock.bash` and `teardown_mock.bash` in `tests` dir.
  - In [dockerized-norlab](https://github.com/norlab-ulaval/dockerized-norlab), its handle by 
    `dn_callback_execute_compose_pre.bash` and `dn_callback_execute_compose_post.bash` in 
    directory `dockerized-norlab-images/core-images/dn-project/`

