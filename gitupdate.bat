# ! /bin/sh
git config --global core.excludesfile /desk/Proj/.gitignore
git add . 
git commit -a -m "update"
git push origin master

pause