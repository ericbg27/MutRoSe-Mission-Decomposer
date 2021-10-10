cd src;

for d in */; do
	cd $d
	rm -rf *.o
	cd ..
done
