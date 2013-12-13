CMAKE_FLAGS= -Wdev -DCMAKE_TOOLCHAIN_FILE=`rospack find rosbuild`/rostoolchain.cmake $(EXTRA_CMAKE_FLAGS)

# make sure we default to all
all:
	rosmake

remake:
	make clean && make

clean_stack:
	@rm -rf build

%:
	@echo "-- >> Building target [ $@ ] for all packages in stack [ $(STACK_NAME) ]..."
	@for package in $$(rosstack contents $(STACK_NAME)); do \
		echo "-- >> Building target [ $@ ] for package [ $$package ]"; \
		if ! ( cd $$(rospack find $$package) && make $@ ); then \
			exit 1; \
		fi; \
		echo "-- << Done building target [ $@ ] for package [ $$package ]"; \
	done

	@if [ "$@" = "clean" ]; then make clean_stack; fi

	@echo "-- << Done building target $@"

STACK_NAME=$$( basename $(PWD))

package_source: all
	$$(rospack find rosbuild)/bin/package_source.py $(CURDIR)
