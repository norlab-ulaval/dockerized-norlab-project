# DNA Command Reference

<div align="center">
<img src="../visual/dna_splash.png" alt="DNA Splash">
</div>

**Dockerized-NorLab project application (DNA)** - A tool for managing Docker-based robotic projects

## Table of Contents

- [Overview](#overview)
- [Global Usage](#global-usage)
- [Commands](#commands)
  - [init](#init) - Initialize a new DNA project
  - [build](#build) - Build DNA Docker images
  - [up](#up) - Start and attach to containers
  - [down](#down) - Stop containers
  - [attach](#attach) - Attach to running containers
  - [exec](#exec) - Execute commands in containers
  - [run](#run) - Run commands in containers
  - [project](#project) - Super project commands
  - [save](#save) - Save Docker images for offline use
  - [load](#load) - Load Docker images from files
  - [config](#config) - Show configuration
  - [version](#version) - Show DNA version
  - [help](#help) - Show help information

## Overview

DNA provides a comprehensive set of commands for managing containerized robotic development environments. Each command is designed to handle specific aspects of the Docker-based workflow, from project initialization to deployment and maintenance.

## Global Usage

```bash
dna COMMAND [OPTIONS]
```

Run `dna COMMAND --help` for detailed information about any specific command.

## Commands

### init
Initialize a new DNA project in the current repository.

**Quick Reference:**
```bash
dna init [OPTIONS]
```

**[→ Detailed Documentation](command/init.md)**

---

### build
Build DNA Docker images for your project.

**Quick Reference:**
```bash
dna build [MODE] [OPTIONS]
```

**[→ Detailed Documentation](command/build.md)**

---

### up
Start containers and attach to them in daemon mode.

**Quick Reference:**
```bash
dna up [OPTIONS]
```

**[→ Detailed Documentation](command/up.md)**

---

### down
Stop and remove containers.

**Quick Reference:**
```bash
dna down [OPTIONS]
```

**[→ Detailed Documentation](command/down.md)**

---

### attach
Attach to a running container.

**Quick Reference:**
```bash
dna attach [OPTIONS]
```

**[→ Detailed Documentation](command/attach.md)**

---

### exec
Execute a command in a running container.

**Quick Reference:**
```bash
dna exec [OPTIONS] COMMAND
```

**[→ Detailed Documentation](command/exec.md)**

---

### run
Run a command in a new container instance.

**Quick Reference:**
```bash
dna run [OPTIONS] COMMAND
```

**[→ Detailed Documentation](command/run.md)**

---

### project
Super project management commands.

**Quick Reference:**
```bash
dna project SUBCOMMAND [OPTIONS]
```

**[→ Detailed Documentation](command/project.md)**

---

### save
Save DNA Docker images to files for offline use.

**Quick Reference:**
```bash
dna save [OPTIONS]
```

**[→ Detailed Documentation](command/save.md)**

---

### load
Load DNA Docker images from files.

**Quick Reference:**
```bash
dna load [OPTIONS]
```

**[→ Detailed Documentation](command/load.md)**

---

### config
Show current DNA configuration (In-progress).

**Quick Reference:**
```bash
dna config [OPTIONS]
```

**[→ Detailed Documentation](command/config.md)**

---

### version
Show DNA version information.

**Quick Reference:**
```bash
dna version
```

**[→ Detailed Documentation](command/version.md)**

---

### help
Show help information for DNA or specific commands.

**Quick Reference:**
```bash
dna help
dna COMMAND --help
```

## Navigation

- [← Back to Main README](../README.md)
- [Installation Guide](install.md)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
- [IDE Integration](ide_integration.md)
