#! /bin/sh


echo "Carrefour" `cat notes.md |grep "^### "|cut -d " " -f 2-|cut -d ":" -f 1|sed -n $1p`
params=`cat notes.md |grep "^### "|cut -d ":" -f 2|sed -n $1p`
cmd="examples/get-crossroad-schematization.py   --osm  -l -o plan.tif  -c $params"
echo 'PYTHONPATH=$PWD' $cmd
PYTHONPATH=$PWD $cmd
gwenview plan.tif
