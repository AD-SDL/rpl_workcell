
#!/usr/bin/env bash



cd ~/workspace/rpl_workcell/color_picker_app

#./color_picker_loop.py --pop_size 8 --exp_budget 80 --solver="Agg"
#./color_picker_loop.py --pop_size 8 --exp_budget 80 --solver="Agg"
#./color_picker_loop.py --pop_size 2 --exp_budget 160
# ./color_picker_loop.py --pop_size 16 --exp_budget 160
# ./color_picker_loop.py --pop_size 32 --exp_budget 160
# ./color_picker_loop.py --pop_size 80 --exp_budget 160
#./color_picker_loop.py --pop_size 128 --exp_budget 160
#./color_picker_application.py --pop_size=64 --exp_budget=128 --solver=Bay --target "[120, 120, 120]"
./color_picker_application.py #--pop_size=32 --exp_budget=128 --solver=Bay --target "[120, 120, 120]"
./color_picker_application.py #--pop_size=16 --exp_budget=128 --solver=Bay --target "[120, 120, 120]"
#./color_picker_application.py --pop_size=2 --exp_budget=128 --solver=Bay --target "[120, 120, 120]"
#./color_picker_application.py --pop_size=1 --exp_budget=128 --solver=Bay --target "[120, 120, 120]"
