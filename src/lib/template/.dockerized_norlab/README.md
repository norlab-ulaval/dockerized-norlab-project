# Dockerized-NorLab project application (DNA)

https://github.com/norlab-ulaval/dockerized-norlab-project.git


## Getting started ... fast
Check [DNA documentation](https://github.com/norlab-ulaval/dockerized-norlab-project?tab=readme-ov-file#documentation) for details

1. Setup/validate `.dockerized_norlab/configuration/` files: 
   - Setup dotenv files: `.env`, `.env.dna` and `.env.local`;
   - Customize files in `build_stage/`;
     - `Dockerfile.project-core-user` should work out of the box for most use cases but it can be customized to take advantage of the Docker cache layer mechanism and the Docker multi-stage build feature;
     - Use `python.requirements-dna.txt` for python packages that you want in the container e.g., package that are not used for release, pinned version for development, jupyter server related. Remark that `python.requirements-dna.txt` can be used simultaniously with a project root level `requirements.txt` file or a `pyproject.toml`;
     - Use `shell.requirements-dna.bash` for installing dependencies from a shell;
   - Customize files in `entrypoints/`. Add project-specific container runtime logic;
   - Check `.dockerized_norlab/configuration/README.md` for more details.
2. From your project `root`, execute the following
   ```shell
   dna help 
   
   # Build your DN-project containers 
   dna build 
   
   # Start your DN-project containers 
   dna up
   
   # Have fun
   # When your done, execute 
   dna down
   ```

