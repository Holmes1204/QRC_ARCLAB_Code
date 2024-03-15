
# Our Modification 

Due to lack of Unitree Go2 robot, we only test the library in our Unitree Go1 robot. We can not ensure that thisc code can work without modification in a new platform like go2, but we hope everything would go well.
```bash
mkdir build
cd build
cmake ..
make moth_go2_arclab
#in the build dir
export ARCLAB_WEIGHT_PATH=../weight
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../lib/arclab_control
./moth_go2_arclab param1 param2
```
According to Unitree develop guide, the `param1` is a network card.And the `param2` might be a new address or something, please contact ICRA 2024 QRC organizing committee for more information.

### Notice
For more reference information, please go to [Unitree Document Center](https://support.unitree.com/home/zh/developer).
