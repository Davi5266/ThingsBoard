#!/bin/bash
docker pull thingsboard/tb-node:4.1.0
docker compose stop thingsboard-ce
docker compose run --rm -e UPGRADE_TB=true thingsboard-ce
#docker compose up -d
