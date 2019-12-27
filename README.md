# Dockerfile cho [Cuộc đua số](https://cuocduaso.fpt.com.vn) 

Môi trường mẫu để cho các đội tham gia cuộc đua số có thể sử dụng. 
Các image được xây dựng chỉ chứa những chương trình và thư viện cơ bản,
các đội tham gia có thể sẽ cần cài đặt thêm một số thư viện cho phù hợp
với phần thi của đội mình

## I. Các [tag](https://hub.docker.com/r/dungpb/dira_ros/tags) được hỗ trợ và `Dockerfile` tương ứng.

- [ros](https://github.com/badungphan99/dira_docker_ros/blob/master/ros/Dockerfile) (image này chỉ chứa ros-melodic)
- [ros-cuda](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-cuda/Dockerfile) (image này có ros-melodic và cuda 10.0, cudnn 7)
- [ros-python](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-python/Dockerfile) (image này có ros và python3, ros kế thừa từ tag `ros`)
- [ros-python-tensorflow](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-python-tensorflow/Dockerfile) (image này có ros, python3, tensorflow cho python image này kế thừa từ tag `ros-cuda`)
- [ros-python2-tensorflow](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-python2-tensorflow/Dockerfile) (image này có ros, python2, tensorflow cho python image này kế thừa từ tag `ros-cuda`)

## II. Cấu trúc thư mục

Cấu trúc thư mục được mô tả như hình sau:
```
<team-name>
├── Dockerfile
├── README.md
\── src
```

### II.1 Dockerfile

Là file để có thể build lên thành 1 image docker.
Trong `Dockerfile` của các đội tham gia buộc phải `FROM` từ [Docker Hub](https://hub.docker.com/r/dungpb/dira_ros) của BTC, 
trường hợp nếu đội thi cần thêm những image khác thì có thể viết một Dockerfile mới 
và gửi để mình có thể đẩy lên docker hub cho các  bạn.

### II.2 README.md

File này các đội cần ghi rõ hướng dẫn chạy cho BTC.

vd: chạy file này trước chờ tới khi xuất hiện thông báo abc.xyz gì đấy thì code đã sẵn sàng để chạy.

### II.3 src

Thư mục chứa toàn bộ source code của các đội.

## III. Cách chạy code

### Bước 0: Cài docker
Chạy script sau (đã được test và chạy trên các hệ điều hành họ Debian):
```bash
curl -fsSL https://get.docker.com | sh && sudo usermod -aG docker $USER
```
**Lưu ý**: Việc chạy `sudo usermod -aG docker $USER` rất quan trọng, đừng bỏ qua nó, nếu không docker sẽ chạy không chính xác.

### Bước 1: Build image

```bash
docker build -t <image-name> .
```

### Bước 2: Chạy 1 image thành container
- chạy docker image không sử dụng gpu:

```bash
docker run --rm -it --network=host -v <đường dẫn đến thư mục src ở trên>:/catkin_ws/src --name <team-name> <image-name> bash
```

- chạy docker image sử dụng gpu:

```bash
docker run --rm -it --gpus=all --network=host -v <đường dẫn đến thư mục src ở trên>:/catkin_ws/src --name <team-name> <image-name> bash
```

Lúc này bạn đã được đưa vào container luôn và có thể sử dụng terminal bình thường.

- Cách để truy cập 1 container đang chạy

```bash
docker exec -it <contaier-name> bash
```

## III. Lưu ý:

- Hai tag `ros` và `ros-cuda` là các image cơ bản, chỉ có các thư viện cơ bản như ros và cuda
```diff
- Các đội bắt buộc phải `FROM` từ image của BTC rồi tự xây dựng thành image của đội mình
```
- Cần file README hướng dẫn chỉ nhằm mục đính thông báo cho BTC code chạy ntn thì sẽ sẵn sàng, các đội cần viết launch file 
để btc có thể chạy theo cú pháp `$ roslaunch team-name team-name.launch`
- option `-v` trong lệnh chạy container là để mount file vào, các đội có thể sử dụng `COPY` từ trong `Dockerfile` cũng được chấp nhận.
