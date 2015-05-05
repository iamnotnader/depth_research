# Dense Depth Maps

This is a library to produce really accurate "dense" depth maps *efficiently*
using two stereo images (but soon arbitrary collections of images).

To see it in action, just do:
$ ./run_tests

Note that you must first install the opencv libraries because I use them to do basic matrix manipulation and image display stuff. You can do this with homebrew on OSX.

## Dynamic Programming Approach
![alt tag](http://i.imgur.com/qexurn3.jpg)

This approach uses dynamic programming to assign depths to pixels on a line-by-line basis, as described in [this paper](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.108.4866&rep=rep1&type=pdf) (slightly simpler than the Ohta-Kanade method). The downside of this approach is that "streaks" appear, since the line-by-line approximation isn't global, and so can't make inter-line adjustments.

## Using Graph Cuts
![alt tag](http://i.imgur.com/qo8WUDJ.jpg)

This approach uses graph cuts to minimize an "energy function" that factors in both pointwise accuracy among correspondences, as well as a smoothness prior. To do the graph cuts, I wrote my own home-grown version of Tarjan+Goldberg's push-relabel algorithm from [their original paper](http://akira.ruc.dk/~keld/teaching/algoritmedesign_f03/artikler/08/goldberg88.pdf) along with some optimizations from [this follow-up paper](http://i.stanford.edu/pub/cstr/reports/cs/tr/94/1523/CS-TR-94-1523.pdf). I also add a couple of my own "experimental" optimizations, such as relabling the source vertex after each global relabel, the correctness of which I have yet to prove (If you try to use this code, be warned it may not work if you don't comment out the "WARNING" sections!). The actual notion of using graph cuts to minimize an energy function defined on the image comes from [this paper](http://www.cs.cornell.edu/rdz/papers/bvz-iccv99.pdf) by Boykov, Veksler, and Zabih. Overall, this method clearly increases the smoothness of the depth map, but leads to some speckel noise that I'm pretty sure is due to its inability to handle occlusions gracefully. With proper tuning, I'm pretty sure this could be fixed but, even given that, the algorithm is so much slower than the dynamic programming approach that it seems more lucrative to just use DP with some smoothing optimizations (such as Ohta Kanade describe), and use multiple images to increase resolution.
