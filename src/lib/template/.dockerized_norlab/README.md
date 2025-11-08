# Dockerized-NorLab project application (DNA)

https://github.com/norlab-ulaval/dockerized-norlab-project.git


## Getting started ... fast
Check [DNA documentation](https://github.com/norlab-ulaval/dockerized-norlab-project?tab=readme-ov-file#documentation) for details

1. Setup/validate `.dockerized_norlab/configuration/` files: 
   - Setup dotenv files: `.env`, `.env.dna` and `.env.local`;
   - Customize files in `build_stage/`;
   - Customize files in `entrypoints/`. Add
      project-specific container runtime logic;
   - Customize `Dockerfile.project-core-user` to fit your need. It should work out of the box for most use cases;
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
