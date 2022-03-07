#include <pybind11/pybind11.h>

namespace py = pybind11;




float some_fn(float arg1, float arg2) {
        return arg1 + arg2;
}


class SomeClass {


        float multiplier;
        public:
        SomeClass(float multiplier_) : multiplier(multiplier_) {};
        float multiply(float input) {
                return multiplier * input;
        }

};

PYBIND11_MODULE(module_name, handle) {

        handle.doc() = "BLAH";
        handle.def("some_fn_python_name", &some_fn);

        py::class_<SomeClass>(handle, "PySomeClass")
                .def(py::init<float>())
                .def("multiply", &SomeClass::multiply);

}
