docker run -it `
--env="DISPLAY=host.docker.internal:0" `
--name insertion `
--net=host `
--privileged `
--mount type=bind,source=$HOME\insertion_shared,target=/home/insertion `
insertion `
bash
    
docker rm insertion
