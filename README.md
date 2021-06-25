# nft_minter_gaka_chu
ROS node for minting NFT for Gaka-Chu. More info https://github.com/airalab/robot_painter/tree/test_branch

This package automatically mints NFT of Gaka-Chu art. 

When path to file is published to topic `/run`, the script takes the name of a file without `.jpg`/`.png` and remembers it.
As soon as filming of the drawing process is finished (see [this](https://github.com/nakata5321/Video_saver_of_Gaka_NFT) for more details), the script waits for 15 secs to let ffmpeg finish all the processes, then renames files (video, picture and rosbag, which initial locations are specified in config) and publishes them to pinata. After that, a metadata is formed and published to IPFS as well. Then the token is issued, using GakaToken json file as contract description. 
There is a testnet option available. 
