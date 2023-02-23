#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/detail/common.h>

namespace py = pybind11;


/*
Safe array is construct for the safest interoperation between numpy as and cpp using pybind.
safe_array is
 - c-contiguous
 - has specified number of dimensions
 - does not do implicit type conversion (e.g. ints to double)
*/

// This suppresses warning "declared with greater visibility than the type of its field"
// https://stackoverflow.com/questions/2828738/c-warning-declared-with-greater-visibility-than-the-type-of-its-field
#pragma GCC visibility push(hidden)

namespace pybind11 { namespace detail {
    /**
     * Struct used to expose the public constructors of unchecked_reference
     * since safe_array requires public constructors or to be labeled as
     * a friend but that means changing the pybind source, so creating a
     * struct and exposing the constructors is the way to go.
     */
    template <typename T, ssize_t Dims>
    struct unchecked_ref_mut : public detail::unchecked_reference<T, Dims> {
        using Self = detail::unchecked_ref_mut<T, Dims>;
        using Base = detail::unchecked_reference<T, Dims>;

        // Constructor for compile-time dimensions:
        template <bool Dyn = Base::Dynamic>
        unchecked_ref_mut(const void *data, const ssize_t *shape, const ssize_t *strides, enable_if_t<!Dyn, ssize_t> dims)
            : detail::unchecked_reference<T, Dims>(data, shape, strides, dims) {}

        // Constructor for runtime dimensions:
        template <bool Dyn = Base::Dynamic>
        unchecked_ref_mut(const void *data, const ssize_t *shape, const ssize_t *strides, enable_if_t<Dyn, ssize_t> dims)
            : detail::unchecked_reference<T, Dims>(data, shape, strides, dims) {}

        auto operator=(Self&& other) -> Self& {
            Self::data_ = other.data_;
            Self::shape_ = other.shape_;
            Self::strides_ = other.strides_;

            return *this;
        }
    };
}}

namespace pybind11 {

/**
 * Copypasta of array_t (mostly constructors with some accessors).
 *
 * NOTE: when a safe_array is specified in a function argument, it looks like
 * pybind calls 'x = safe_array()' (the default constructor) to initialize it,
 * then proceeds to call 'y = safe_array(h, borrowed_t{})' but this one has the
 * numpy array data. An equal `x.operator=(y)` call is then made to assigned
 * the initialized x with the data. x is then passed into function that specifies
 * the safe_array<T, Dims>&.
 *
 * TODO: could potentially get rid of the unchecked_reference variable, since
 * we're already plundering so much of its functions/fields so much that
 * it's not worth the extra abstraction.
 */
template <typename T, ssize_t Dims>
struct safe_array_mut: public array {
    static_assert(!detail::array_info<T>::is_array, "Array types cannot be used with safe_array_mut");

    using Self = safe_array_mut<T, Dims>;
    using value_type = T;
    static constexpr bool Dynamic = false;  // safe array has fixed number of dimensions

private:
    detail::unchecked_ref_mut<T, Dims> uref;

    struct private_ctor {};
    // Delegating constructor needed when both moving and accessing in the same constructor
    safe_array_mut(private_ctor, ShapeContainer &&shape, StridesContainer &&strides, const T *ptr, handle base)
      : array(std::move(shape), std::move(strides), ptr, base)
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { }

public: /* Constructors */

    // Copy constructor for self
    safe_array_mut(const Self& other)
      : array(other)
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { assert_dimensions(); }

    safe_array_mut()
      : array(0, static_cast<const T *>(nullptr))
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { /* e don't assert dimensions for due to it being the default init */ }

    safe_array_mut(handle h, borrowed_t)
      : array(h, borrowed_t{})
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { assert_dimensions(); }

    safe_array_mut(handle h, stolen_t)
      : array(h, stolen_t{})
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { assert_dimensions(); }

    safe_array_mut(const object &o)
      : array(raw_safe_array_mut(o.ptr()), stolen_t{})
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    {
        assert_dimensions();
        if (!m_ptr) throw error_already_set();
    }

    explicit safe_array_mut(const buffer_info& info)
      : array(info)
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { assert_dimensions();}

    safe_array_mut(ShapeContainer shape, StridesContainer strides, const T *ptr = nullptr, handle base = handle())
      : array(std::move(shape), std::move(strides), ptr, base)
      , uref(array::data(), array::shape(), array::strides(), array::ndim())
    { assert_dimensions(); }

    explicit safe_array_mut(ShapeContainer shape, const T *ptr = nullptr, handle base = handle())
      : safe_array_mut(
            private_ctor{},
            std::move(shape),
            detail::c_strides(*shape, itemsize()),
            ptr, base)
    { assert_dimensions();}

public: /*Array accessors */

    template<typename... Ix>
    auto index_at(Ix... index) const -> ssize_t {
        return offset_at(index...) / itemsize();
    }

    constexpr ssize_t itemsize() const {
        return sizeof(T);
    }

    template<typename... Ix> auto data(Ix... index) const -> const T* {
        return static_cast<const T*>(array::data(index...));
    }

    template<typename... Ix> auto mutable_data(Ix... index) -> T* {
        return static_cast<T*>(array::mutable_data(index...));
    }

    static bool check_(handle h) {
        const auto &api = detail::npy_api::get();
        return api.PyArray_Check_(h.ptr())
            && api.PyArray_EquivTypes_(detail::array_proxy(h.ptr())->descr, dtype::of<T>().ptr())
            && detail::check_flags(h.ptr(), detail::npy_api::NPY_ARRAY_C_CONTIGUOUS_)
            && detail::array_proxy(h.ptr())->nd == Dims;
    }

public: /* Accessors taken from unchecked_reference and unchecked_mutable_reference */

    void assert_dimensions() {
        if (Dims >= 0 && Self::ndim() != Dims)
            throw std::domain_error("safe_array has incorrect number of dimensions: "
                + std::to_string(Self::ndim())
                + "; expected "
                + std::to_string(Dims));
    }

    auto operator=(Self&& other) -> Self& {
        Self::uref = std::move(other.uref);
        array::operator=(std::move(other));
        return *this;
    }


    template <typename... Ix>
    auto operator()(Ix... index) const -> const T& {
        return Self::uref(index...);
    }

    /// Mutable, unchecked access to data at the given indices.
    template <typename... Ix>
    auto operator()(Ix... index) -> T& {
        static_assert(ssize_t{sizeof...(Ix)} == Dims || Dynamic,
                "Invalid number of indices for unchecked array reference");
        return const_cast<T &>(Self::uref(index...));
    }

    template <ssize_t D = Dims, typename = detail::enable_if_t<D == 1 || Dynamic>>
    auto operator[](ssize_t index) const -> const T& {
        return operator()(index);
    }

    /**
     * Mutable, unchecked access data at the given index; this operator only participates if the
     * reference is to a 1-dimensional array (or has runtime dimensions).  When present, this is
     * exactly equivalent to `obj(index)`.
     */
    template <ssize_t D = Dims, typename = detail::enable_if_t<D == 1 || Dynamic>>
    auto operator[](ssize_t index) -> T& {
        return operator()(index);
    }

protected:
    /// Create array from any object -- always returns a new reference
    static PyObject *raw_safe_array_mut(PyObject *ptr) {
        if (ptr == nullptr) {
            PyErr_SetString(PyExc_ValueError, "cannot create a pybind11::safe_array_mut from a nullptr");
            return nullptr;
        }
        PyObject * result = detail::npy_api::get().PyArray_FromAny_(
            ptr, dtype::of<T>().release().ptr(), 0, 0,
            detail::npy_api::NPY_ARRAY_ENSUREARRAY_, nullptr);
        if (result == nullptr) {
            throw error_already_set();
        }
        return result;
    }
};

template<typename T>
safe_array_mut<T, 1> zeros(ssize_t dim0) {
    safe_array_mut<T, 1> result({dim0});
    memset(result.mutable_data(), 0, sizeof(T)*result.size());
    return result;
}

template<typename T>
safe_array_mut<T, 2> zeros(ssize_t dim0, ssize_t dim1) {
    safe_array_mut<T, 2> result({dim0, dim1});
    memset(result.mutable_data(), 0, sizeof(T)*result.size());
    return result;
}

template<typename T>
safe_array_mut<T, 3> zeros(ssize_t dim0, ssize_t dim1, ssize_t dim2) {
    safe_array_mut<T, 3> result({dim0, dim1, dim2});
    memset(result.mutable_data(), 0, sizeof(T)*result.size());
    return result;
}

template <typename T>
using safe_array_dyn = typename pybind11::array_t<T, pybind11::array::c_style>;

template <class T, ssize_t Dims>
using safe_array = const typename py::safe_array_mut<T, Dims>;
} // namespace pybind11

namespace pybind11 { namespace detail {
    template <typename T, ssize_t Dims>
    struct pyobject_caster<safe_array_mut<T, Dims>> {
        using type = safe_array_mut<T, Dims>;

        bool load(handle src, bool convert) {
            // ignore 'convert' for safety
            if (!type::check_(src))
                return false;
            value = type::ensure(src);
            return static_cast<bool>(value);
        }

        static handle cast(const handle &src, return_value_policy /* policy */, handle /* parent */) {
            return src.inc_ref();
        }
        PYBIND11_TYPE_CASTER(type, _("contiguous ") + handle_type_name<type>::name);
    };
}}

#pragma GCC visibility pop
