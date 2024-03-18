# docu-build

the `sphinx-build` *must* works recursively.

> SOLUTION: **symlink** `index.rst` and `conf.py`  \
> to be available on `dev_ws/src` level

```bash
doc_root=~/dev_ws/src
cd ${doc_root}
ln -s bautiro/doc/conf_py   conf.py
ln -s bautiro/doc/index_rst index.rst
```

## plain sphinx build

```bash
# interactive
docker run -u $(id -u ${USER}):$(id -g ${USER}) -it -v /home/${USER}/dev_ws/src:/docs mysphinx bash
```

## docker sphinx build

see [Dockerfile](Dockerfile) \
Build your own docker-image from `sphinxdoc/sphinx`
and extend with `READ-THE_DOCS` theme and `MARKDOWN` (myst) support

```docker
FROM sphinxdoc/sphinx
RUN  pip install --upgrade myst-parser sphinx-autobuild sphinx-rtd-theme
```

```bash
docker build . -t mysphinx
```

```bash
# interactive usage
$ docker run -u $(id -u ${USER}):$(id -g ${USER}) -it -v /home/${USER}/dev_ws/src:/docs mysphinx bash
# from inside container
root@mysphinx:/docs# sphinx-build -Eaq . out


# one shot way
$ docker run -u $(id -u ${USER}):$(id -g ${USER})  -v /home/${USER}/dev_ws/src:/docs mysphinx sphinx-build -Eaq . out
```
