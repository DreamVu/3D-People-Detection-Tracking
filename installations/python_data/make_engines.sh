if ! command -v trtexec &> /dev/null
then
    FILE=/usr/src/tensorrt/bin/trtexec
    if test -f "$FILE"; then
        export PATH=$PATH:/usr/src/tensorrt/bin
    else
        echo "Requirements not complete. Installing..."
        FOLDER=/usr/src/tensorrt/samples/trtexec/
        if [ -d "$FOLDER" ]; then
            sudo chown -R $USER: /usr/src/tensorrt/
            cd /usr/src/tensorrt/samples/trtexec/
            make
            cd -
            if test -f "$FILE"; then
                export PATH=$PATH:/usr/src/tensorrt/bin
            else
                echo "Can't locate trtexec. run: 'sudo find / -name trtexec' and add it to the PATH"
                exit
            fi
        else
            echo "Can't locate trtexec folder. run: 'sudo find / -name tensorrt' and find the folder with trtexec. Go in that folder and run 'make'"
            exit
        fi
    fi
fi

a=1

success=""
failed=""
while [ $a -lt 6 ]
do
    echo "generating engine for $a"
    
    b=$a.trt

    trtexec --onnx=./generate/$a.onnx --fp16 --batch=1 --saveEngine=/usr/local/bin/data/$b --workspace=$1

    if [ $? -eq 0 ]; then 
        rm ./trt/$b 
        echo "$b engine created"
        success="$success$b "
    else
        echo "$b engine creation failed"
        $failed="$failed$b "
    fi
    a=`expr $a + 1`
done

trtexec --onnx=./generate/6.bin --fp16 --saveEngine=/usr/local/bin/data/engine_hqdec.trt --workspace=$1

sudo chown -R dreamvu:dreamvu /usr/local/bin/data/*.trt 

echo "the following engines were built successfully: $success"

