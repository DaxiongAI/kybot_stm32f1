.PHONY: all cmake cmake_do install clean distclean tags ut stat prepare mail publish_on_target 
.PHONY: uninstall_makefile install_makefile build_make hex bin

# please read:
# ../out/x86/debug/makefile
include $(bin_root)/make_rule_conf.mk


SRCS:=$(shell find $(src_root)/src  -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" \))
INCS:=$(shell find $(src_root)/inc -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" \))
#UNITS:=$(shell find $(src_root)/utest  -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.h" \))
BINS:=$(shell find $(src_root)/bin  -type f \( -name "*" \))

# The execute path
exec_root:=$(build_root)/bin/
# 名字与CMakeLists.txt含义一样
project_root=$(shell readlink -f $(bin_root)/..)
# 这里'(' ')'需要转义？cmake里不需要？
project_name=$(shell cd $(project_root); pwd | sed 's/^\(.*\)[/]//')
#project_name=$(shell echo $(project_root) | awk -F'/' '{print $$1}')
#project_name=$(shell  echo "/home/zwb/stm32f10x_test" | awk '{split($0,b,"/") print b[2]}')
#project_name=$(shell str=${'readlink -f $(bin_root)/..'} ; echo -n ${'$$str##*/'})
#project_name=$(shell $(project_root)##*/)
#project_name=$(filter "##*/",$(project_root))
#project_name=$(dir $(project_root))

all: build_make hex bin

# 进入build目录，使用cmake生成的makefile编译
build_make:$(build_root)
	@cd $(build_root) && make
	
# 生成hex文件	
hex:$(build_root)
	@cd $(build_root) && make $(project_name).hex
# 生成bin文件
bin:$(build_root)
	@cd $(build_root) && make $(project_name).bin

# below code cause cmake always invoked for $(build_root) is a variable instead 
# of a real path.
# $(build_root): cmake


debug:
	@echo "project_root=" $(project_root)
	@echo "project_name=" $(project_name)
	@echo "SRCS=" $(SRCS)
	@echo "INCS=" $(INCS)
	@echo "UNITS=" $(UNITS)
	@echo "BINS=" $(BINS)
	@echo "top=" $(top)
	@echo "build_root=" $(build_root)
	@echo "src_root=" $(src_root)
	@echo "bin_root=" $(bin_root)


$(build_root): $(make_root)/CMakeLists_config.txt 
	mkdir -p $(build_root)

cmake_do: $(build_root) 
	@#-j5, use 5 thread parallel build
	@#cd $(build_root) && cmake $(src_root) && make -j5 && make install
	@#cd $(build_root) && cmake -DSTM32_CHIP=stm32f103c8 -DCMAKE_TOOLCHAIN_FILE=$(src_root)/cmake/gcc_stm32.cmake -DCMAKE_BUILD_TYPE=Debug $(src_root)/
	@#cd $(build_root) && cmake -DCMAKE_TOOLCHAIN_FILE=$(src_root)/cmake/gcc_stm32.cmake -DCMAKE_BUILD_TYPE=Debug $(src_root)/
	cd $(build_root) && cmake -DCMAKE_BUILD_TYPE=Debug $(src_root)
#cmake: cmake_do install
cmake: cmake_do 

$(install_root):
	mkdir -p $(install_root)
install: $(install_root)
	cd $(build_root) && make install -s

#$(bin_root)/make_rule_conf.mk: $(bin_root)/make_rule_conf.mk.templet
$(bin_root)/make_rule_conf.mk: 
	#cp -f $@.templet $@
	ifeq ($(makefile_install_dir), )
		#(warn("the makefile install dir is NULL"))
	endif

insmf: $(bin_root)/make_rule_conf.mk
	cd $(bin_root) && ./install_makefile.sh $(src_root) $(makefile_install_dir)

uninsmf: $(bin_root)/make_rule_conf.mk
	cd $(bin_root) && ./uninstall_makefile.sh $(src_root) $(makefile_install_dir)

clean:
	cd $(make_root) && cd .. && rm -rf ./build/

distclean:
	cd $(make_root) && cd .. && rm -rf ./build/
	@#cd $(make_root) && rm -rf set_env.sh
	@#cd $(bin_root) && rm -rf publish_info.txt && ./uninstall_makefile.sh $(src_root)

tags: $(src_root)/tags
$(src_root)/tags: $(SRCS) $(INCS) $(UNITS) $(BINS)
	@cd $(src_root) && ctags -R --extra=+q src/ inc/ lib/  


$(make_root)/set_env.sh: $(make_root)/set_env.sh.templet
	cp -f $@.templet $@


$(bin_root)/set_env.sh: $(bin_root)/set_env.sh.templet
	cp -f $@.templet $@

$(bin_root)/target.mk : $(bin_root)/target.mk.templet
	cp -f $@.templet $@

download: all $(make_root)/set_env.sh $(bin_root)/set_env.sh
	# ref: http://stackoverflow.com/questions/11076350/how-do-you-export-a-variable-through-shell-script
	# below code is error, export do NOT work in child shell process.
	#@cd $(bin_root) && ./set_env.sh
	#@cp -f $(bin_root)/set_env.sh $(make_root)/set_env.sh
	#@cd $(bin_root) && ./set_env.sh
	@cd $(make_root) && ./new_window_download.sh



# publish exec binary to <publish computer's directory>
# publish on target.mk setting.
publish_on_target: distclean cmake all install 
	#@echo "publish " $(build_root)
	@cd $(bin_root) && ./exec_publish.sh $(exec_root)


stat:
	$(eval tmp_stat := $(shell mktemp -d))
	@echo $(tmp_stat)
	@cd $(src_root) && gitstats . $(tmp_stat) && firefox $(tmp_stat)/index.html &


prepare:
	sudo yum install cmake
	sudo yum install gcc-c++
	sudo yum install gitstats

mail:
	@cd $(bin_root) && ./mail_backup.sh
