# LIBELAS

This code incorporates the [LIBELAS](http://www.cvlibs.net/software/libelas/) library in the following way:
- header files in [stereo-vision/lib/include/iCub/stereoVision/elas](./lib/include/iCub/stereoVision/elas) (this directory)
- source files in [stereo-vision/lib/src/elas](./lib/src/elas)
- the original Matlab wrappers and example images have not be incorporated 
- the library gets compiled within the compilation of the main `stereo-vision` repository, no separate compilation is needed

We include, in this directory:
- the original [README](./lib/include/iCub/stereoVision/elas/LIBELAS-README.txt) from [LIBELAS](http://www.cvlibs.net/software/libelas/), which indicates the code LICENSE and reference papers
- LIBELAS [LICENSE](./lib/include/iCub/stereoVision/elas/LICENSE), which we downloaded as the [plain text form](http://www.gnu.org/licenses/gpl-3.0.txt) of the [GNU General Public License (GPLv3)](http://www.gnu.org/licenses/gpl.html)