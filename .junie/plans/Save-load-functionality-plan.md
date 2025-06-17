
# General instructions
Follow the instructions in `.junie/Guideline.md`.

# Main goal
Implement a _save docker image to file_ and a _load docker image from file_ functionality for offline use.
The intended use cases: (case 1) deployment in remote area i.e., build image localy and physicaly transfer the image to an embed computer without requiring internet access; (case 2) faster build i.e., build image localy and transfer the image to a remote host which have limited computation capabilities.

# Implementation details

Add `dnp build [OPTIONS] --save DIRPATH [develop|deploy]` new flag to `build` command with `DIRPATH` being the save directory path and make it only available for `develop` and `deploy` services.
Add new command `load` following signature `dnp load [OPTIONS] FILEPATH [develop|deploy]` with `FILEPATH` being the path of the file to load and make it only available for `develop` and `deploy` services.
Under the hood, use `docker image save --output <path/to/tar/archive/name>.tar` and `docker image load --input <path/to/tar/archive/name>.tar` commands (Docker image references: https://docs.docker.com/reference/cli/docker/image/save/ and https://docs.docker.com/reference/cli/docker/image/load/).
Load and save functionalities should collect and save to file the relevant components so that `dnp [up|run] [develop|deploy]` can work offline on remote host.
The output of `dnp build --save DIRPATH deploy` should be like this:
```markup
dnp-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── .dockerized_norlab_project/
  │   ├── configuration/
  │   │   ├── project_entrypoints/
  │   │   │   ├── project-deploy/
  │   │   │   ├── dn_entrypoint.global.attach.callback.bash
  │   │   │   └── dn_entrypoint.global.init.callback.bash
  │   │   ├── .env
  │   │   ├── .env.dnp
  │   │   └── .env.local
  │   ├── dn_container_env_variable/
  │   └── .env.PLACEHOLDER_SUPER_PROJECT_NAME
  ├── artifact/                 <- empty directory
  ├── external_data/            <- empty directory
  ├── meta.txt
  └── <tar/archive/name>.tar
```
The output of `dnp build --save DIRPATH develop` should be like this:
```markup
dnp-save-develop-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── meta.txt
  └── <tar/archive/name>.tar
```
In the deploy case, we are assuming that the project repository is not cloned and that we don't have internet access to re-build the image so the the saved solution need to be protable and self-contained. 
In the develop case, we are assuming that the project repository is cloned on the remote host.
A `meta.txt` should be created on save and it should gather important information such as the `DNP_CONFIG_SCHEME_VERSION`, `DN_PROJECT_GIT_REMOTE_URL` environment variables name and value, the repository branch/tag name and the date and time at saving.

# Update and refactoring
Update `dnp` entrypoint at `src/bin/dnp` and update its test file.
You will need to extend `dnp::super_project_dnp_sanity_check()` to cover the case where `dnp` is executed offline from a `dnp-save-deploy-<SUPER_PROJECT_REPO_NAME>-<image-tag>` saved deploy directory.
You will need to extend `dnp::load_super_project_configurations()` with an option to manualy pass a value to `super_project_git_remote_url` env var so that we can bypass the `git remote get-url origin` when `dnp` is used offline.
Add a offline mode to `dnp::import_lib_and_dependencies()` to manualy pass value to `git_project_path` and bypass `git rev-parse --show-toplevel`.

# Testing
Implement the corresponding the tests files.
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
