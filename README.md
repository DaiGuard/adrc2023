# adrc2023

Auto Drive RC: 自動運転ラジコン2023のリポジトリ

![](docs/adrc2023.png)

---

## 全体構成

![](docs/function_block.png)

## シーケンス図

![](docs/sequence_diagram.png)

##

```bash
docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:foxy serial --dev /dev/ttyTHS0 
```
