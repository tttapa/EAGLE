#!/usr/bin/env bash

targetpc="vierre64"
generations=100
make -j$(nproc) tuner
rm *.png
scp bin/tuner r0663636@ssh.esat.kuleuven.be:~/Private/EAGLE-bin
echo "Connecting to $targetpc over ssh"
ssh -l r0663636 -X ssh.esat.kuleuven.be "ssh -X $targetpc \"source ~/.bash_profile && cd ~/Private/EAGLE-bin && ./tuner --no-show --load \\\$HOME/Private/EAGLE-Params --generations $generations\""
# TODO: why does this segfault?
# if [ $? = 0 ]
# then
    scp r0663636@ssh.esat.kuleuven.be:~/Private/EAGLE-bin/*.png .
    scp r0663636@ssh.esat.kuleuven.be:~/Private/EAGLE-bin/tuner.output .
# fi