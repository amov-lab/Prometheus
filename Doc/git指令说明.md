2. submodule 更新

若项目中包含.gitmodules, 进入到git项目根目录下，输入命令, .gitmodules中的所有项目都会进行更新:

git clone 父项目.git
git submodule init
git submodule update (update时，submodule分支必须已在正确分支上)

submodule远程分支发生变更后，直接使用git submodule update是不会进行更新操作的。必须依次进入到各个submodule的目录，进行git pull操作，如果submodule数目很多，每次发版本时必须进入所有目录进行git pull，这将是噩梦。不过有个更简单的方法，

git submodule foreach git checkout master
git submodule foreach git pull


此时还需要父仓库指向新的游离分支，直接commit 提交即可。


## token问题

git remote set-url origin https://<your_token>@github.com/amov-lab/Prometheus.git
