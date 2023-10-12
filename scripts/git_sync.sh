cd /home/rpl/workspace
pwd
for i in *
do
  cd /home/rpl/workspace
  cd $i
  echo $i
  git pull
  cd ..
done
