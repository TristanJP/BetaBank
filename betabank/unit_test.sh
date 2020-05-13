echo "======================"
echo "RUNNING UNIT TESTS"
echo
python -m unittest src.betabank.test_frame_analyser
python -m unittest src.betabank.test_main

sleep 1
echo
echo "======================"
echo "FINISHED TESTS"