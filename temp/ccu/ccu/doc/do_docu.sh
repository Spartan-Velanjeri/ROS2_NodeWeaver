# Copyright 2023 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

#!/bin/bash
if [ ! -f "./conf.py" ]; then
    echo "'conf.py' not in current directory"
    exit
fi

docker run --rm -it -v $PWD:/docs \
                    -u $UID \
            --network host \
            bcr-de01.inside.bosch.cloud/bautiro/sphinx_pypipa \
            sphinx-autobuild -aE /docs /docs/OUT
