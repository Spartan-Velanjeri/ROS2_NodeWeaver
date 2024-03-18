PATH_MAPS=${BTR_WS}/workspaces/galactic/develop/btr_ws/src/rpm/lu_rough_localization

if [ -d ${PATH_MAPS}/maps ]; then
    echo ""
    echo "Directory allready existing:";
    echo ${PATH_MAPS};
    echo $(du -h -d 0 maps)
    echo ""
else
    echo ""
    echo "Downloading to";
    echo ${PATH_MAPS}/maps;
    echo ""

    cd ${PATH_MAPS};

    echo "Please provide a token for https://artifactory.boschdevcloud.com"
    wget --recursive --no-parent --user $(whoami) --ask-password https://artifactory.boschdevcloud.com/artifactory/BautiroArtifacts-local/lu_rough_localization/maps/;
    mv artifactory.boschdevcloud.com/artifactory/BautiroArtifacts-local/lu_rough_localization/maps maps;
    rm artifactory.boschdevcloud.com/ -r;
fi

