
# General instructions
Follow guidelines at `.junie/Guidelines.md`.

# Main goal
Implement a _save docker image to file_ and a _load docker image from file_ functionality for offline use.
The intended use cases: (case 1) deployment in remote area i.e., build image localy and physicaly transfer the image to an embed computer without requiring internet access; (case 2) faster build i.e., build image localy and transfer the image to a remote host which have limited computation capabilities.


# Highlevel
Add new command `dnp save [OPTIONS] DIRPATH SERVICE` with `DIRPATH` being the save directory path.
Add new command `dnp load [OPTIONS] DIRPATH/dnp-save-<SERVICE>-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>`.
Add new save flag to `build` command following signature `dnp build [OPTIONS] --save DIRPATH [develop|deploy]` which call command `dnp save [OPTIONS] DIRPATH [develop|deploy]` and make it only available for `develop` and `deploy` services.

# Implementation details
The `.tar` file name will be generated using the follow the pattern `<DN_PROJECT_IMAGE_NAME>-SERVICE.<PROJECT_TAG>.tar` with `DN_PROJECT_IMAGE_NAME` and `PROJECT_TAG` env var subtitution and `SERVICE` being one of `develop` or `deploy`.
Under the hood, use `docker image save --output <path/to/tar/archive/name>.tar IMAGE` with `IMAGE` being the image name and `docker image load --input <path/to/tar/archive/name>.tar` commands (Docker image references: https://docs.docker.com/reference/cli/docker/image/save/ and https://docs.docker.com/reference/cli/docker/image/load/).
Load and save functionalities should collect and save to file relevant components so that `dnp [up|run] [develop|deploy]` can work offline on remote host once `dnp load ...` executed.

The output of `dnp build --save DIRPATH deploy` should be like this:
```markup
dnp-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── <SUPER_PROJECT_REPO_NAME>/
  │   ├── .dockerized_norlab_project/
  │   │   ├── configuration/
  │   │   │   ├── project_entrypoints/
  │   │   │   │   ├── project-deploy/
  │   │   │   │   ├── dn_entrypoint.global.attach.callback.bash
  │   │   │   │   └── dn_entrypoint.global.init.callback.bash
  │   │   │   ├── .env
  │   │   │   ├── .env.dnp
  │   │   │   └── .env.local
  │   │   ├── dn_container_env_variable/
  │   │   └── .env.<SUPER_PROJECT_NAME>
  │   ├── .git/
  │   ├── artifact/                 <- empty directory
  │   └── external_data/            <- empty directory
  ├── meta.txt
  └── <DN_PROJECT_IMAGE_NAME>-deploy.<PROJECT_TAG>.tar
```

The output of `dnp build --save DIRPATH develop` should be like this:
```markup
dnp-save-develop-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── meta.txt
  └── <DN_PROJECT_IMAGE_NAME>-develop.<PROJECT_TAG>.tar
```

In the deploy case, we are assuming that the project repository is not cloned on the remote host while in the develop case, we are assuming that the project repository is cloned on the remote host.
In both cases, we are assuming that we don't have internet access to re-build the image so the saved solution need to be portable.
Aditionaly, the deploy case solution need to be self-contained.

A `meta.txt` should be created on save and it should gather important information such as the `DNP_CONFIG_SCHEME_VERSION`, `DN_PROJECT_GIT_REMOTE_URL`, `DN_PROJECT_ALIAS_PREFIX` environment variables name and value, the repository branch/tag name and the date and time at saving.

Command `dnp load ...` when it load a deploy image, should `cd` at `dnp-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/` root once the image is loaded. 
Command `dnp load ...` when it load a develop image, should execute alias `dnp-<DN_PROJECT_ALIAS_PREFIX>-cd` on remote host once the image is loaded. 


# Update and refactoring
Update `dnp` entrypoint at `src/bin/dnp`.

# Testing
Implement the corresponding the tests files.
Add an integration tests script that validate the full pipeline: e.g., `dnp build --save DIRPATH deploy` -> to file at `DIRPATH` -> `dnp load DIRPATH/dnp-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>` 
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Has instructed in `.junie/Guidelines.md`, both unit-tests and integration tests must pass.
