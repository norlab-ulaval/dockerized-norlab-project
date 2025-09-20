# DNA Configuration Files

Refer to [DNA documentation](https://github.com/norlab-ulaval/dockerized-norlab-project?tab=readme-ov-file#documentation) on
  _Project Initialization & Configuration_, section _Configuration Files_ for details.


## DNA dotenv file loading precedence

1. .env.dna
2. .env
3. .env.local
4. .env.dna-internal (DNA repo)

## ★ Note On Configuration Changes 

- Rebuild and restart container after modifying requirement files i.e., `dna build && dna down && dna up`
- Restart container after modifying entrypoints i.e., `dna down && dna up`

## External References

- [Docker-Compose environment variables precedence](https://docs.docker.com/compose/how-tos/environment-variables/envvars-precedence/)
- Docker predefined environment variables
  - [Docker](https://docs.docker.com/reference/cli/docker/)
  - [Docker-Compose](https://docs.docker.com/compose/how-tos/environment-variables/envvars/)

