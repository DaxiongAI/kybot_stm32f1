# !/bin/sh
# 在src各个子目录下生成makefile文件
baseDirForScriptSelf=$(cd "$(dirname "$0")"; pwd)
echo "full path to currently executed script is : ${baseDirForScriptSelf}"
uninstall_makefile(){
        need_uninstall $1
        for_each_dir $1
}
need_uninstall(){
        #echo "needtouch-cd $1"
        cd $1
        #res=$(ls | egrep '.[cChH]$|.([cC][pP][pP]|[cC][xX][xX]|[hP][pP][pP]|[hH][xX][xX])$')
        res=$(ls | egrep '\.[cChH]$|\.([cC][pP][pP]|[cC][xX][xX]|[hP][pP][pP]|[hH][xX][xX])$')
        #echo "res=$res"
        if [ "$res" = "" ]
        then
                #echo "have no source"
                return
        fi

        echo "directory $1 has source file, cd in it to uninstall makefile"
        if [ -f makefile ] 
        then
                rm makefile
        fi
}

for_each_dir(){
        # 遍历参数1 
        # echo "$1"
        # ls $1
        #echo "foreach-cd $1"
        #cd $1
        #pwd
        for file in $(ls)
        do
                #echo "dir is $file"
                # 如果是目录...，然后继续遍历，递归调用
                if [ -d $file ]
                then
                        #echo "subdir is $file"
                        uninstall_makefile $file
                        #need_touch $file
                        #for_each_dir $file
                        # 返回上一级目录，否则不会递归
                        cd ..
                fi
        done
        return 0
}
# 执行，如果有参数就遍历指定的目录，否则遍历当前目录
echo "uninstall makefile in $# dirs"
# 参数1：root dir
# 其他参数： 子目录
if [ $# = 0 ]
then
        echo "please set the arg"
        exit
elif [ $# = 1 ]
then

        uninstall_makefile $1
        #need_touch $1
        #for_each_dir $1
else
        # echo "arg1 is $1"
        for arg in "$@"
        do
                # 忽略父目录
                if [ "$arg" != "$1" ]
                then
                        #echo "arg = $arg"
                        uninstall_makefile $1/$arg
                        #need_touch $1/$arg
                        #for_each_dir $1/$arg
                fi
        done
        #n=$#
        #for((i=2; i<=n; i++))
        #do
         #       echo "subdir:${lists[i]}"
         #       need_touch "$1/${lists[i]}"
         #       for_each_dir "$1/${lists[i]}"
        #done
fi 

