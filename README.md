# Docker Cuộc đua số
Để đảm bảo môi trường chạy tối ưu với mã nguồn của các đội, ban tổ chức đã xây dựng một docker image mẫu để các đội có thể pull về. Docker này chứa hầu hết các thư viện cần thiết để xây dựng một mã nguồn đủ khả năng chạy được thử thách mô phỏng dựa trên mã nguồn của các đội thi các năm trước. Các đội thi sẽ pull image này về để có môi trường chạy code và thông báo cho ban tổ chức các yêu cầu về thư viện thêm vào. Ban tổ chức yêu cầu các đội thi thường xuyên gửi mã nguồn và yêu cầu về thư viện, bao gồm cả phiên bản của thư viện thích hợp để đảm bảo mã nguồn được sử dụng trong ngày thi chạy ổn định. Tài liệu này nhằm mục đích hướng dẫn các thí sinh cách sử dụng docker. Khi bạn sử dụng tài liệu này, ban tổ chức ngầm định rằng bạn đã hiểu cách sử dụng Terminal Ubuntu và các tạo cũng như chạy package ROS. 

## I. Docker là gì? 
Docker là một ứng dụng cô lập và khởi tạo môi trường chạy mã nguồn được thiết kế để chạy một mã nguồn bất kì trên bất kì máy tính nào mà không phải cân nhắc về môi trường. Các thư viện và thiết lập môi trường được lưu trên docker image tạo ra từ Dockerfile sẽ được sử dụng để tạo ra một container, môi trường chạy mã nguồn đảm bảo đồng nhất và biệt lập với môi trường máy tính chủ. Thí sinh có thể tìm hiểu về Docker qua [tài liệu](https://github.com/fpt-corp/DigitalRace-Docker/blob/master/Docker%20tutorial) sau. 
Ban tổ chức sẽ cung cấp cấu trúc thư mục chuẩn khi thi. Đội thi được khuyên giữ nguyên cấu trúc thư mục này để hạn chế sự cần thiết phải chỉnh sửa các đường dẫn, thuận tiện cho việc hướng dẫn. Thí sinh có thể download thư mục mẫu tại [đây](https://github.com/fpt-corp/DigitalRace-Docker)
## II. Cấu trúc bài thi
Cấu trúc thư mục:
```
team1___
        |___
	|---<thư mục code (catkin/src) của thí sinh.
        |---Dockerfile
```
Trong cấu trúc thư mục này, thí sinh cần đặt source code vào trong folder /<Tên-đội>. Source code đặt vào trong folder này là một package ROS đầy đủ. Khi luyện tập ở nhà các đội tự đổi tên các folder trùng với tên đội mình.
Đội thi đổi <tên đội> theo tên đội của mình. Các thiết lập bên dưới nhằm giới hạn tài nguyên tính toán mà docker container có thể dùng để chạy. Các giới hạn hiện tại là tối đa 6Gb RAM và 4Ghz, tương đương 80% năng lực tính toán của CPU Core i7 8700K. Đây là giới hạn phần cứng chính thức của vòng thi này. Các đội cần tối ưu thuật toán để đảm bảo chạy tốt trong giới hạn này.
## III. Cách chạy code
### Bước 1: Build image
Image được build từ dockerfile trong chính thư mục mẫu của ban tổ chức. Thí sinh được quyền sửa file này tùy theo nhu cầu của mình. Lưu ý thí sinh chỉ được FROM các image mà ban tổ chức đã công bố (trong file mẫu).
___
Trong trường hợp thí sinh phải cài thêm nhiều thư viện có thể build image riêng trên chính image của ban tổ chức. Sau đó thí sinh push file đó lên DockerHub.
```
sudo docker build --tag=<tên image> .
```
Trong cuộc thi, tên image sẽ trùng với tên của đội thi.

### Bước 2: Chạy container

```
sudo docker run -it --gpus=all --network=host --memory=6000M -d -v ~/<đường dẫn đến folder DigitalRace-Docker>/team1:/root/catkin_ws/src/  --name <tên đội> diradocker/dira_env:first_stable bash
```

Nếu đội muốn giữ nguyên container sau khi thoát. Nhìn chung không khuyến khích do    container chiếm bộ nhớ, và các thay đổi về thư viện nên được ghi lại trong Dockerfile. Chỉ dùng nếu đội thi cần một thư viện xung đột với thư viện của ban tổ chức. Trong trường hợp đó cần giữ nguyên container vào thông báo cho ban tổ chức để tạo docker image phù hợp cho đội thi.
```
sudo docker run --rm -it --gpus=all --network=host --memory=6000M -d -v ~/<đường dẫn đến folder DigitalRace-Docker>/team1:/root/catkin_ws/src/ --name <tên đội> diradocker/dira_env:first_stable bash
```
Nếu đội thi muốn xóa container sau khi thoát. Được khuyến khích

Để mở terminal container sau khi chạy
```
sudo docker exec -it <tên đội> bin/bash
```
Lưu ý: Tên đội ở đây chính là tên container vừa tạo của đội
___
Chạy một trong hai lệnh trên sẽ tạo container và cho thí sinh truy cập vào container. Thí sinh thao tác trong container như với Terminal Ubuntu bình thường. Do không có công cụ GUI để kiểm tra các file trong directory của Docker container, thí sinh được khuyên sử dụng lệnh `ls` thường xuyên để biết trong directory đang có các thư mục nào. Container tạo ra từ image của ban tổ chức đã có sẵn ROS Melodic với catkin_ws nằm tại /root. Thư mục /root/catkin_ws/src trong container được kết nối trực tiếp tới ~/<đường dẫn đến folder DigitalRace-Docker>/team1/<tên đội> trên máy chủ, do đó trong container hiện đã có source code do đội thi đặt vào ở các bước trên. Đội thi nếu đã thực hiện tất cả các bước như trên, có thể chạy lệnh catkin_make ngay và thử chạy mã nguồn của mình để kiểm tra xem có thiếu thư viện nào không. Đội thi được khuyến khích cài đặt các thư viện nếu thực sự cần thiết. Nếu có nhu cầu thêm thư viện chưa cài sẵn, thí sinh nên cài đặt trong container trước để kiểm tra sự hiệu quả, sau đó thêm vào trong file Dockerfile. Vào ngày thi, đội thi nhất thiết phải có một Dockerfile đầy đủ của phiên bản image mà mình đang sử dụng, nếu không ban tổ chức không thể xây dựng được môi trường giống với môi trường các bạn dùng để chạy code của mình. Hướng dẫn viết Dockerfile có thể xem ở [đây](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/). Trong trường hợp đội thi cần một phiên bản thư viện xung đột với phiên bản thư viện có sẵn trên Docker, đội thi được khuyến khích thông báo sớm cho ban tổ chức để có thể xây dựng một docker image thích hợp. Nếu đội thi không thông báo trước thời hạn quy định, ban tổ chức sẽ không chịu trách nhiệm về các trường hợp xung đột thư viện dẫn đến lỗi.

## IV. Cách nộp bài
Thí sinh nộp thư mục có cấu trúc như đã nói ở trên. Ban tổ chức sẽ build lại trên máy của btc container đó.
BTC sẽ catkin_make trong container là chạy file launch:
VD đội 123 - roslaunch team123 team123.launch

___
### *Lưu ý trước khi thi
Sau khi đã thiết lập xong Dockerfile để có một container chạy mã nguồn không bị lỗi, đội thi cần gửi mã nguồn và Dockerfile cho ban tổ chức để đảm bảo có thể chạy được trong ngày thi, chậm nhất là 1 tuần trước ngày thi. Nếu sau thời hạn này, đội thi phát sinh nhu cầu cài đặt thêm thư viện, đội thi phải tự chịu những rủi ro có thể có. Ban tổ chức vẫn liên tục tiếp nhận yêu cầu kiểm tra Dockerfile và tạo image Docker cho các thư viện bị xung đột sau hạn chót 1 tuần, tuy nhiên không đảm bảo việc mã nguồn có thể chạy tốt. Những lệnh chạy trong hướng dẫn này chỉ nhằm chạy được docker container dùng 1 lần, nếu thí sinh muốn lưu container lại thì cần xem kĩ hướng dẫn sử dụng docker ở đầu. 
Thí sinh build image riêng dựa trên hai tag base của BTC (ros hoặc ros-cuda) cần làm thêm một bước là push DockerFile đó lên DockerHub cá nhân và gửi cho BTC kiểm tra 1 tuần trước khi thi.
