---
title:  "Keyword auto with Eigen library"
date:   2018-06-25 22:31:00 +0200
categories: General 
---

Today I encountered a weird problem when using `Eigen` library with keyword `auto`. The scenario is as follows:

{% highlight c++ %}
Eigen::Affine3d aff; 

// aff is assigned with a valid value
// ...

auto r = aff.rotation();
auto res = r * r;

{% endhighlight %}

In the end, the result variable `res` contains a `3x3` matrix with only zero values, no matter what values `aff` has. After a bit of googling, I found out some relevant issues and `Eigen` documentation:
* [Eigen auto type deduction in general product][link 1]
* [Eigen and C++11 type inference fails for Cholesky of matrix product][link 2]
* [Lazy Evaluation and Aliasing][link 3]

I still have not quite understand this issue. As a temporary solution, using `auto` with `Eigen` library should be avoided as much as possible.

I will post another article once I understand it fully. 


[link 1]: [https://stackoverflow.com/questions/26705446/eigen-auto-type-deduction-in-general-product]
[link 2]: [https://stackoverflow.com/questions/27113261/eigen-and-c11-type-inference-fails-for-cholesky-of-matrix-product]
[link 3]: [https://eigen.tuxfamily.org/dox/TopicLazyEvaluation.html]


