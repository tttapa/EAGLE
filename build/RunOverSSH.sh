#!/usr/bin/env bash

targetpc="pc-klas3-3"
make -j$(nproc) tuner
scp bin/tuner r0663636@ssh.esat.kuleuven.be:~/Private/EAGLE-bin
ssh -l r0663636 -X ssh.esat.kuleuven.be "ssh -X $targetpc \"source ~/.bash_profile && cd ~/Private/EAGLE-bin && ./tuner --load \\\$HOME/Private/EAGLE-Params --generations 2\""
scp r0663636@ssh.esat.kuleuven.be:~/Private/EAGLE-bin/*.png .