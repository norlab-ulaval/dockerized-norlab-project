
# General instructions
Follow guidelines at `.junie/guidelines.md`.

# Main goal
Implement a _save docker image to file_ and a _load docker image from file_ functionality for offline use.
The intended use cases: (case 1) deployment in remote area i.e., build image localy and physicaly transfer the image to an embed computer without requiring internet access; (case 2) faster build i.e., build image localy and transfer the image to a remote host which have limited computation capabilities.


# Highlevel
Add new command `dna save [OPTIONS] DIRPATH SERVICE` with `DIRPATH` being the save directory path.
Add new command `dna load [OPTIONS] DIRPATH/dna-save-<SERVICE>-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>`.
Add new save flag to `build` command following signature `dna build [OPTIONS] --save DIRPATH SERVICE` which call command `dna save [OPTIONS] DIRPATH SERVICE` with `SERVICE` being restricted to `develop` or `deploy` only.

# Implementation details
Under the hood, use `docker image save --output <path/to/tar/archive/name>.tar IMAGE` with `IMAGE` being the image name and `docker image load --input <path/to/tar/archive/name>.tar` commands (Docker image references: https://docs.docker.com/reference/cli/docker/image/save/ and https://docs.docker.com/reference/cli/docker/image/load/).
The `.tar` file name will be generated using the follow the pattern `<DN_PROJECT_IMAGE_NAME>-SERVICE.<PROJECT_TAG>.tar` with `DN_PROJECT_IMAGE_NAME` and `PROJECT_TAG` env var subtitution and `SERVICE` being one of `develop` or `deploy`.
Load and save functionalities should collect and save to file relevant components so that `dna [up|run] [develop|deploy]` can work offline on remote host once `dna load ...` is executed.

The output of `dna build --save DIRPATH deploy` should be like this:
```markup
dna-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── <SUPER_PROJECT_REPO_NAME>/
  │   ├── .dockerized_norlab/
  │   │   ├── configuration/
  │   │   │   ├── project_entrypoints/
  │   │   │   │   ├── project-deploy/
  │   │   │   │   ├── dn_entrypoint.global.attach.callback.bash
  │   │   │   │   └── dn_entrypoint.global.init.callback.bash
  │   │   │   ├── .env
  │   │   │   ├── .env.dna
  │   │   │   └── .env.local
  │   │   ├── dn_container_env_variable/
  │   │   └── .env.<SUPER_PROJECT_NAME>
  │   ├── .git/                     <- make a full copy of the .git directory
  │   ├── artifact/                 <- empty directory
  │   └── external_data/            <- empty directory
  ├── meta.txt
  └── <DN_PROJECT_IMAGE_NAME>-deploy.<PROJECT_TAG>.tar
```

The output of `dna build --save DIRPATH develop` should be like this:
```markup
dna-save-develop-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/
  ├── meta.txt
  └── <DN_PROJECT_IMAGE_NAME>-develop.<PROJECT_TAG>.tar
```

In the deploy case, we are assuming that the project repository is not cloned on the remote host while in the develop case, we are assuming that the project repository is cloned on the remote host.
In both cases, we are assuming that we don't have internet access to re-build the image so the saved solution need to be portable.
Aditionaly, the deploy case solution need to be self-contained.

A `meta.txt` should be created on save and it should gather important information such as the `DNA_CONFIG_SCHEME_VERSION`, `DN_PROJECT_GIT_REMOTE_URL`, `DN_PROJECT_ALIAS_PREFIX` environment variables name and value, the repository branch/tag name and the date and time at saving.

Command `dna load ...` when it load a deploy image, should `cd` at `dna-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>/` root once the image is loaded. 
Command `dna load ...` when it load a develop image, should execute alias `dna-<DN_PROJECT_ALIAS_PREFIX>-cd` on remote host once the image is loaded. 


# Update and refactoring
Update `dna` entrypoint at `src/bin/dna` with the new available commands.

# Testing
Implement the corresponding the tests files.
Add an integration tests script that validate the full pipeline: e.g., `dna build --save DIRPATH deploy` -> to file at `DIRPATH` -> `dna load DIRPATH/dna-save-deploy-<SUPER_PROJECT_REPO_NAME>-<date><hour><minute>` 
Create at least one test case per command argument and/or options.
Test relevant option and arguments combinaison.
Has instructed in `.junie/guidelines.md`, both unit-tests and integration tests must pass.
