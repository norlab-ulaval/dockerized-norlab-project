# README improvement plan

Goal: improve the repository documentation. 

## General instructions
Review the code base.
Dont execute `dna COMMAND`, read the implementation instead. Each script implementation, command implementation and shell function implementation are well documented. 
Code documentation are provided either using a commented text documentation header or using a `DOCUMENTATION_*` environment variable which contains the help documentation displayed to user when using an `--help` flag.

## Writing instructions
Use clear and concise sentences.
Use technical terminology when necessary.
Link to official documentation when refering to _docker_, _docker compose_ or _buildx_.
Add a Markdown TOC on semanticaly grouping page e.g., `dna` command documentation -> `dna build` command page, `dna up` command page, `dna exec` command page, ... with `dna` command documentation being the semantic group page.   
Add navigation link on each markdown pages so that user can easily and intuitively know where they are and how to go where they want.
Add diagram, directory tree structure or code example when its relevant. A picture is worth a thousand word. 

## Documentation implementation tests instructions
Tests each hyperlink for broken link, including link pointing to external page.
Validate that each hyperlink go to the expected page/section.

---

## Main readme sections instructions 
The expected sections order should be the following:
- Repository title
- _Description_
- _What it does_
- _Getting started_
  - _Install Dockerized-NorLab Project app `dna`_
  - _Make a repository ready to use with DNA_
  - _Create, run and stop DN container_
- _Why_
- _Use cases_
- Repository wide TOC with link to other documentation pages

### The _What it does_ section
Review the _What it does_ section and make sure it does not miss key point.

### The _Getting started_ section 
- Improve the current install instructions if necessary.
- This section should be short and cover the minimum necessary to start using the application.
- Add a note and link where to find detail configuration instructions and command documentation in the doc _Getting started_.

### Mission statement related sections (_Why_ and _Use cases_)
Those sections goal are to give the user an overview and sell them the benefit of using _Dockerized-NorLab project application (DNA)_ and _Dockerized-NorLab (DN)_.
By analogie, its like an "elevator pitch"

#### _Why_ section 
Write a _Why_ section:
- The _Why_ section should give a short summary of DNA and DN purposes.
- It should highlight the problems that DNA and DN are solving and explain why it matters. 
- It should cover the essential points without being exhaustive, the goal is to sell, not to explain in details.
- Cover at least the following points: 
  - Key properties ❯ reproducibility;
  - Key properties ❯ isolation;
  - Project Collaboration;
  - Fast Deployment;
  - Code Quality.
Inspire yourself with the pictures in `visual/DN_presentation_diagram/` that are related to _Why_ section.
Use the pictures in `visual/DN_presentation_diagram/` directory for the related _Why_ section.

#### _Use cases_ section
Write a _use cases_ section with the following use cases:
  - Local and remote development (develop mode) ⇒ imply DNA is installed on the local and remote host;
  - Deployment (deploy mode) ⇒ imply DNA is installed on the target host;
  - Release ⇒ imply a git repository release tag was created and imply DNA is not require to run DN container;
  - Running _continuous integration_ test;
  - Running _slurm_ jobs;
  - Working on multiple OS and architecture: `l4t/arm64` (jetson), `darwin/arm64` (Mac OsX) and `linux/x86`;
  - GPU support through Nvidia-docker.
Inspire yourself with the pictures in `visual/DN_presentation_diagram/` that are related to _Use case_ section.
Use the pictures in `visual/DN_presentation_diagram/` directory for the related _Use cases_ section.
  
## Command documentation
Write a _Command documentation_ pages covering each `dna COMMAND`, one markdown page per command. 
Consolidate command documentation markdown pages in a new `documentation/command/` directory.
Organize the command documentation so that each command has its own dedicated page, all linked to a main `documentation/dna.md` command documentation page with TOC.
Use their respective help documentation content.
Cover all relevant options, arguments, flag and usage instruction as mentioned in their respective help documentation.
Add example usage with use typical use cases.

## Exaustive install documentation
Add a `documentation/install.md` page.
Write details install instruction with the relevant install casse options covered by `install.bash` script.
Add relevant install information for each supported os/aarch host.

## Exaustive project initialization and configuration documentation
Add a `documentation/project_initialization_and_configuration.md` page.
Write instructions on how to DN initialize a project using `dna` app.
Write instructions on how to configure and customize DN for each project particular need using files in `.dockerized_norlab` directory.
Explain the purposes of each new directories and files added by `dna init` command.
Review the content of `src/lib/template/.dockerized_norlab/README.md` and `src/lib/template/.dockerized_norlab/configuration/README.md`.

## IDE integration documentation
Add a `documentation/ide_integration.md` page.
Review the content of section _Setup PyCharm IDE_ in `src/lib/template/.dockerized_norlab/README.md`.
Add instructions on how to configure a remote development workflow.
Add instructions for integrating DNA and DN with an IDE or other development softwares.

## General notes:
The command related to the release use case are not implemented yet but they are planned and in progress. They are an key future feature so its important to talk about them in the main readme. 
The _develop_ and _deploy_ mode are both _interactive_ mode and use platform specific run configuration.
_ci-tests_ run and _slurm_ jobs run are non-interactive mode.
 
