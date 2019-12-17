# Docker [cuộc đua số](https://cuocduaso.fpt.com.vn) 

Môi trường mẫu để cho các đội tham gia cuộc đua số có thể sử dụng. 
Các image được xây dựng chỉ chứa những môi trường cơ bản,
các đội tham gia cần cài đặt thêm một số thư viện cho phù hợp với 
phần thi của đội mình

## I. Các `tag` được hỗ trợ và `Dockerfile` tương ứng.

- [ros](https://github.com/badungphan99/dira_docker_ros/blob/master/ros/Dockerfile)
- [ros-cuda](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-cuda/Dockerfile)
- [ros-python](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-python/Dockerfile)
- [ros-python-tensorflow](https://github.com/badungphan99/dira_docker_ros/blob/master/ros-python-tensorflow/Dockerfile)

## II. Cấu trúc thư mục

Cấu trúc thư mục được mô tả như hình sau:
```
<team-name>
├── Dockerfile
├── README.md
├── src
```

### II.1 Dockerfile

Là file để có thể build lên thành 1 image docker.
Trong `Dockerfile` của các đội tham gia buộc phải `FROM` từ docker hub của BTC, 
trường hợp nếu đội thi cần thêm những image khác thì có thể viết một Dockerfile mới 
và gửi để mình có thể đẩy lên docker hub cho các  bạn.

### II.2 README.md

File này các đội cần ghi rõ hướng dẫn chạy cho BTC.

vd: chạy file này trước chờ tới khi xuất hiện thông báo abc.xyz gì đấy thì code đã sẵn sàng để chạy.

### II.3 src

Thư mục chứa toàn bộ source code của các đội.

## III. Cách chạy code

### Bước 1: Build image

`docker build -t <image-name> .`

### Bước 2: Chạy 1 image thành container
- chạy docker image không sử dụng gpu:

`docker run --rm -it --network=host -v <đường dẫn đến thư mục src ở trên>:/catkin_ws/src --name <team-name> <image-name> bash`

- chạy docker image sử dụng gpu:

`docker run --rm -it --gpus=all --network=host -v <đường dẫn đến thư mục src ở trên>:/catkin_ws/src --name <team-name> <image-name> bash`

Lúc này bạn đã được đưa vào container luôn và có thể sử dụng terminal bình thường.

- Cách để truy cập 1 container đang chạy

`docker exec -it <contaier-name> bash`

## III. Lưu ý:

