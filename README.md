# cartographer_2d

## 简介
这个 repository 旨在让更多人看到后可以更好的理解 google cartographer 这个项目的代码，不过由于我的项目主要是用到 2d 这一块，所以我删减掉了 mapping_3d 这个文件夹的内容，并将代码中少量用到3d的部分挪到了 mapping_2d 目录中，这部分可以对比 google cartographer 来看。

这里用的是 cartographer 的 0.3.0 这个 release 版本，后面的 commit 我会一直跟进，不过在下一个 release 版本出来之前基本上不会出现文件结构的变化，同时在这个 release 版本之后 google 的开发人员给cartographer加入了很多 gRPC 的功能，这个对于我来说并没有实际的用处，所以暂时没有跟进这方面代码的打算。

最后，google 的 cartographer 项目在算法上并没有很复杂，可以说只有一些进步的空间，这一部分也是我会自己添加的，当然是参考一些新的论文，找到一些优秀的思路添加到代码中。
