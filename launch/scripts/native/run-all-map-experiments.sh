ok1=0 #d1 semi
ok2=0 #d1 static
ok3=1 #d1 om
ok4=0 #d8 semi
ok5=1 #d8 static
ok6=1 #d8 om



while [ $ok1 -eq 1 ] || [ $ok2 -eq 1 ] || [ $ok3 -eq 1 ] || [ $ok4 -eq 1 ] || [ $ok5 -eq 1 ] || [ $ok6 -eq 1 ]
do
    echo "ok1 $ok1"
    echo "ok2 $ok2"
    echo "ok3 $ok3"
    echo "ok4 $ok4"
    echo "ok5 $ok5"
    echo "ok6 $ok6"

    if [ $ok1 -eq 1 ]
    then
        ./run-map-experiment.sh d1 semi
        ok1=$?
    fi
    if [ $ok2 -eq 1 ]
    then
        ./run-map-experiment.sh d1 static
        ok2=$?
    fi
    if [ $ok3 -eq 1 ]
    then
        ./run-map-experiment.sh d1 om
        ok3=$?
    fi
    if [ $ok4 -eq 1 ]
    then
        ./run-map-experiment.sh d8 semi
        ok4=$?
    fi
    if [ $ok5 -eq 1 ]
    then
        ./run-map-experiment.sh d8 static
        ok5=$?
    fi
    if [ $ok6 -eq 1 ]
    then
        ./run-map-experiment.sh d8 om
        ok6=$?
    fi
done

echo "ok1 $ok1"
echo "ok2 $ok2"
echo "ok3 $ok3"
echo "ok4 $ok4"
echo "ok5 $ok5"
echo "ok6 $ok6"
echo "all experiments run succesfully!"