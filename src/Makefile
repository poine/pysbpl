pysbpl.so: sbpl.pyx setup_sbpl_ext.py
	@echo "## Building sbpl python extension ##"
	python setup_sbpl_ext.py build_ext --inplace
	@echo "## all done##\n"


clean:
	rm -rf *~ *.pyc sbpl.cpp pysbpl.so build *.mprim

