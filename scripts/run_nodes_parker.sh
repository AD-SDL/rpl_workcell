#!/usr/bin/env bash

ssh rpl@parker.cels.anl.gov

cd ~/workspace
git clone https://github.com/ad-sdl/rpl_workcell
cd rpl_workcell/docker
docker compose -f parker-nodes.compose.yaml up -d

